# terminal.py
import serial
import time
import os
import zlib
from typing import List, Tuple
import json
import math

BOOTLOADER_SIZE=0x10000 #64kb

PACKET_DATA_MAX_LENGTH = 1024

def read_version():
    try:
        with open('version.json', 'r') as f:
            version_data = json.load(f)
            major_version = version_data.get('major', 0)
            minor_version = version_data.get('minor', 0)
            print(f"Read version: {major_version}.{minor_version}")
            return f"{major_version}.{minor_version}"
    except (FileNotFoundError, json.JSONDecodeError) as e:
        print(f"Error reading version file: {e}")
        return "0.0"


class Packet:
    def __init__(self,crc32:int,data:bytes):
        self.crc32=crc32
        self.data=data

    def to_bytes(self) -> bytes:
        crc32_bytes = self.crc32.to_bytes(4, byteorder='big')
        data_bytes = bytearray(PACKET_DATA_MAX_LENGTH)
        data_bytes[:len(self.data)] = self.data

        packet_bytes = crc32_bytes + bytes(data_bytes)
        print(f"Packet bytes: {packet_bytes.hex()}")
        return packet_bytes
    

class BootloaderTerminal:
    def __init__(self, port: str = 'COM3', baudrate: int = 115200):
        try:
            self.serial = serial.Serial(port, baudrate, timeout=1.0)
            self.current_version = read_version()  # Should be stored/managed better
            self.kernel_binary = "output.bin"
            self.data_size = PACKET_DATA_MAX_LENGTH
            self.packet_size = 6+8+self.data_size
            self.max_retries = 3
            print(f"Connected to {port} at {baudrate} baud")
        except serial.SerialException as e:
            print(f"Error opening serial port: {e}")
            raise SystemExit
        except Exception as e:
            print(f"Unexpected error: {e}")
            raise SystemExit

    def calculate_crc32(self, data: bytes) -> int:
        return zlib.crc32(data) & 0xFFFFFFFF

    def send(self, data: bytes) -> bool:
        print(f"Sending: {data}")
        self.serial.write(data)
        return True


    def read_firmware(self,binary_path):
        try:
            with open(binary_path, 'rb') as f:
                firmware_data = f.read()
            print(f"Before padding: {len(firmware_data)} bytes")
            bytes_to_pad=BOOTLOADER_SIZE-len(firmware_data)
            print(f"Padding bootloader with {bytes_to_pad} bytes")
            if bytes_to_pad<0:
                raise ValueError("Bootloader is too big")
            else:
                padding=bytes([0xFF]*(bytes_to_pad))
            firmware_data+=padding
                
        except FileNotFoundError:
            print(f"Error: Could not find {binary_path}")
            return False
        return firmware_data


    def get_firmware_size(self,firmware_data:bytes):
        return len(firmware_data)
    

    def send_firmware_size(self,file_size:int):
        self.send(f"{file_size}".encode())
        while True:
            if self.serial.in_waiting:
                response = self.serial.readline().decode().strip()
                print(f"Received response: {response}")
                return response
            time.sleep(0.1)

    

    def send_and_receive(self,data):
        print("Data size in bytes: ",len(data))
        # crc32 = self.calculate_crc32(data)
        # header = f"{len(data):06b}:{crc32:08X}"
        # packet = header.encode() + data
        # packet = Packet(self.calculate_crc32(data),data).to_bytes()

        self.serial.write(data)
        # self.serial.write(packet)
        # print(f"Sent packet: {data.hex()}")  # Print hex format
        while True:
            if self.serial.in_waiting:
                response = self.serial.readline().decode().strip()
                # print(f"Received response length: {len(response)}")
                # print(f"Raw received bytes: {response.encode().hex()}")
                print(f"Decoded response: {response}")
                if not response.startswith("ACK"):
                    continue
                return response
            time.sleep(0.1)


    def send_firmware_packet(self, data,iter) -> bool:
        recv = self.send_and_receive(data)
        print(f"ACK: {recv}")
        if recv == f"ACK {iter}":
            return True
        return False

    def run(self):
        # print("Terminal started. Waiting for bootloader...")
        while True:
            try:
                # print("Checking for messages...")
                if self.serial.in_waiting:
                    print("Message received")
                    encoded_message = self.serial.readline()
                    # print("Encoded: ",encoded_message)
                    message = encoded_message.decode(errors='replace').strip()
                    print(f"Received: {message}")

                    
                    if message.startswith("VERSION_REQ:"):
                        bootloader_version = message.split(":")[1]
                        print(f"Current version: {self.current_version}")
                        print(f"Bootloader version: {bootloader_version}")
                        
                        self.send(f"{self.current_version}".encode())
                        continue

                    elif message.startswith("FIRMWARE_REQ:"):
                        print("Received firmware request")
                        firmware_data = self.read_firmware(self.kernel_binary)
                        if not firmware_data:
                            print("Firmware read failed")
                            continue
                        file_size = self.get_firmware_size(firmware_data)
                        print(f"Sending firmware of size {file_size} bytes")
                        recv=self.send_firmware_size(file_size)
                        print(recv)
                        time.sleep(0.1)
                        iteration = math.ceil(file_size/self.data_size)
                        print(f"Total iteration: {iteration}")

                        i=0
                        while(i<iteration):
                            print(f"Sending firmware packet {i+1}/{iteration}")
                            res = self.send_firmware_packet(firmware_data[i*self.data_size:(i+1)*self.data_size],i)
                            if not res:
                                print("Firmware update failed")
                                break
                            i+=1
                        print("Firmware update successful")
                        continue

                time.sleep(0.1)

            except serial.SerialException as e:
                print(f"Serial communication error: {e}")
                break
            except Exception as e:
                print(f"Unexpected error: {e}")
                break

if __name__ == "__main__":
    terminal = BootloaderTerminal()
    try:
        terminal.run()
    except KeyboardInterrupt:
        print("\nTerminal stopped by user")
    finally:
        terminal.serial.close()