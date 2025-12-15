import serial.tools.list_ports as lp
import serial
import time
from functools import reduce
import operator
import struct



class Terminal:
    def __init__(self, help_text = 'helpFiles/help.txt'):
        self.help_text = ''
        with open(help_text, 'r') as f:
            self.help_text = f.read()
        self.ports = [port.device for port in lp.comports()]
        self.end_marker = '\f'
        self.arduino = None
        self.velocity = None
        self.log = ''

    def set_connection(self, port : int, baurdate : int):
        print('trying to connect')
        self.arduino = serial.Serial(port, baurdate, timeout=1)

        time.sleep(5)
        response = self.arduino.readline().decode().strip()
        print(f'Response: {response}')
        self.log += f'Response: {response}\n'


    def send_command(self, command_char, magnitude=0):
        char_bytes = command_char.encode()

        mgn_bytes = struct.pack('<f', magnitude)
        cmd_bytes = char_bytes + mgn_bytes
        check_sum_int = reduce(operator.xor, cmd_bytes)
        check_sum_bytes = struct.pack('<h', check_sum_int)
        cmd = cmd_bytes + check_sum_bytes + self.end_marker.encode()
        self.arduino.write(cmd)

        time.sleep(5)
        response = self.arduino.readline().decode().strip()
        print(f'Response: {response}')
        self.log += f'{command_char} {magnitude}\n'
        self.log += f'Response: {response}\n'

    def save_log(self, file_name='log.txt'):
        try:
            with open(file_name, 'w') as f:
                f.write(self.log)
            print('Log saved')
        except IOError:
            print(f'Failed to save log to {file_name}')

    def help(self):
        print(self.help_text)



def check_input(input_text, predicate, range_l=None, range_u=None):
    while True:
        input_variable = input(input_text)
        pred = predicate(input_variable) if range_l is None\
                            else predicate(input_variable, range_l, range_u)
        if pred:
            return input_variable
        else:
            print('Invalid input for data')

def my_pred(s, m, n):
    try:
        s_num = float(s)
        return s_num >= m and s_num <= n
    except ValueError:
        return False

def accept_command(term, options):
    entered_data = ''
    while entered_data != 'quit':
        entered_data = input(options)

        match entered_data.lower():
            case 'e':
                print(f'Available ports: {term.ports}')
                port = check_input('Enter port: ', lambda s: s in term.ports)
                baudRate = check_input('Enter baud rate: ', my_pred, 300, 115200)
                term.set_connection(port, baudRate)
            case 'c':
                if term.arduino is None:
                    print('You must establish connection first')
                    continue
                print('Commands: m, r, v, s, b, i, P, I, D, g, S, back')
                command = input('Enter command: ')

                match command:
                    case 'm' | 'r':
                        m, n = (-200, 200) if command.lower() == 'r' else (-200, 200)
                        magnitude = int(check_input(
                            'Enter the magnitude: ', my_pred, m, n))
                        term.send_command(command, magnitude)
                    case 'v':
                        m, n = (0, 254)
                        magnitude = int(check_input(
                            'Enter the magnitude: ', my_pred, m, n))
                        term.velocity = magnitude
                        term.send_command(command, magnitude)
                    case 's' | 'b' | 'i':
                        term.send_command(command)
                    case 'P':
                        magnitude = float(check_input(
                            'Enter the magnitude for kp: ', my_pred, 0.0, 30.0))
                        term.send_command(command, magnitude)
                    case 'I':
                        magnitude = float(check_input(
                            'Enter the magnitude for ki: ', my_pred, 0.0, 30.0))
                        term.send_command(command, magnitude)
                    case 'D':
                        magnitude = float(check_input(
                            'Enter the magnitude for kd: ', my_pred, 0.0, 30.0))
                        term.send_command(command, magnitude)
                    case 'g':
                        term.send_command(command)
                    case 'S':
                        term.send_command(command)
                    case _:
                        print('Invalid command')

            case 'help':
                term.help()
            case 'quit':
                break
            case 'h':
                print(term.log)
            case 's':
                term.save_log(input('Enter filename: '))
            case _:
                print('Invalid input for command')

def main():
    options = 'Options to enter:\ne\nc\nh\ns\nquit\nhelp \nYour option: '
    term = Terminal()
    try:
        accept_command(term, options)
    finally:
        if term.arduino:
            term.arduino.close()

if __name__ == '__main__':
    main()




