import re
import numpy as np
import datetime
import tabulate

# Define the regular expression for parsing CAN log lines
log_line_regex = re.compile(r'\((\d+\.\d+)\)\s+(\w+)\s+(\w+)#(\w+)')
node_id_bits = 7

def print_and_format(parsed_data, filename):
    table_data = [
        [
            datetime.datetime.fromtimestamp(entry['timestamp']).strftime('%Y-%m-%d %H:%M:%S'),  # Formatted timestamp
            entry['interface'], 
            entry['can_id'], 
            entry['function_code'], 
            entry['node_id'], 
            entry['data']
        ]
        for entry in parsed_data
    ]

    headers = ["Timestamp", "Interface", "CAN ID", "Function Code", "Node ID", "Data"]
    
    table = tabulate.tabulate(table_data, headers=headers, tablefmt="grid")
    
    with open(log_file_path + '_output.txt', 'w') as outFile:
        print(table, file=outFile)
    
def convert_function_code(code, nodeId):
    match code:
        case 0:
            return "NMT"
        case 1:
            return "NMT" if (nodeId == 0) else "EMCY"
        case 2:
            return "TIME"
        case 3:
            return "Transmit PDO 1"
        case 4:
            return "Recieve PDO 1"
        case 5:
            return "Transmit PDO 2"
        case 6:
            return "Recieve PDO 2"
        case 7:
            return "Transmit PDO 3"
        case 8:
            return "Recieve PDO 3"
        case 9:
            return "Transmit PDO 4"
        case 10:
            return "Recieve PDO 4"
        case 11:
            return "Transmit SDO"
        case 12:
            return "Recieve SDO"
        case 13:
            return "HEARTBEAT"

def split_string(s, y):
    # First string: all but the last y characters
    first_part = s[:-y] if y <= len(s) else ''
    # Second string: the last y characters
    second_part = s[-y:] if y <= len(s) else s
    return first_part, second_part

def parse_can_log(file_path):
    parsed_data = []

    # Open the log file and parse each line
    with open(file_path, 'r') as log_file:
        for line in log_file:
            match = log_line_regex.match(line.strip())
            if match:
                timestamp = float(match.group(1))  # Extract timestamp
                interface = match.group(2)         # Extract CAN interface
                can_id = match.group(3)            # Extract CAN ID
                data = match.group(4)              # Extract CAN data

                # Split CAN ID into function node and node ID
                # Assuming CAN ID is hexadecimal and function node is the first nibble (1st character)
                can_id_bin = np.base_repr(int(can_id, 16), base = 2)
                function_code_bin, node_id_bin = split_string(can_id_bin, node_id_bits)
                try:
                    function_code = int(function_code_bin, 2)
                    node_id = int(node_id_bin, 2)
                except:
                    print("Invalid")
                    function_code = "N.A."
                    node_id = "N.A."
                
                function_code = convert_function_code(function_code, node_id)

                # function_node = can_id[:1]  # First nibble (4 bits)
                # node_id = can_id[1:]        # Remaining bits

                parsed_data.append({
                    'timestamp': timestamp,
                    'interface': interface,
                    'can_id': can_id,
                    'function_code': function_code,
                    'node_id': node_id,
                    'data': data
                })

    return parsed_data

log_file_path = '/home/zoe/Desktop/candump-2025-08-16_203946.log'  # Replace with your log file path
parsed_can_data = parse_can_log(log_file_path)

print_and_format(parsed_can_data, log_file_path)