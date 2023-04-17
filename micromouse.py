import serial
import time
import math

def flood_fill(maze, start, goal_area):
    maze[start[0]][start[1]] = 0
    queue = [start]

    while queue:
        x, y = queue.pop(0)
        for dx, dy in [(-1, 0), (1, 0), (0, -1), (0, 1)]:
            nx, ny = x + dx, y + dy
            if 0 <= nx < len(maze) and 0 <= ny < len(maze[0]) and maze[nx][ny] != -1:
                new_value = maze[x][y] + 1
                if maze[nx][ny] == 0 or maze[nx][ny] > new_value:
                    maze[nx][ny] = new_value
                    queue.append((nx, ny))

    path = []
    x, y = None, None

    min_val = float('inf')
    for gx, gy in goal_area:
        if maze[gx][gy] < min_val:
            min_val, x, y = maze[gx][gy], gx, gy

    while (x, y) != start:
        path.append((x, y))
        neighbors = [(x-1, y), (x+1, y), (x, y-1), (x, y+1)]
        min_val, next_cell = float('inf'), None
        for nx, ny in neighbors:
            if 0 <= nx < len(maze) and 0 <= ny < len(maze[0]) and maze[nx][ny] != -1 and maze[nx][ny] < min_val:
                min_val, next_cell = maze[nx][ny], (nx, ny)
        x, y = next_cell

    path.reverse()
    return path

def send_phase_change_to_robot(ser):
    command = 'S'
    ser.write(command.encode())
    time.sleep(0.5)

def init_maze(rows, cols):
    maze = [[1000000] * cols for _ in range(rows)]
    return maze

def update_position_and_orientation(sensor_dict, current_position, initial_position, initial_heading, wheel_radius, wheel_distance, wheel_ticks):
    ticks_left = sensor_dict["wheelCountLeft"]
    ticks_right = sensor_dict["wheelCountRight"]

    distance_left = (ticks_left / wheel_ticks) * 2 * math.pi * wheel_radius
    distance_right = (ticks_right / wheel_ticks) * 2 * math.pi * wheel_radius

    delta_distance = (distance_right + distance_left) / 2
    delta_theta = (distance_right - distance_left) / wheel_distance

    x, y = initial_position
    if initial_heading == "north":
        x -= delta_distance * math.cos(delta_theta)
        y -= delta_distance * math.sin(delta_theta)
    elif initial_heading == "south":
        x += delta_distance * math.cos(delta_theta)
        y += delta_distance * math.sin(delta_theta)
    elif initial_heading == "east":
        x += delta_distance * math.sin(delta_theta)
        y -= delta_distance * math.cos(delta_theta)
    elif initial_heading == "west":
        x -= delta_distance * math.sin(delta_theta)
        y += delta_distance * math.cos(delta_theta)

    heading_rad = math.atan2(y, x)
    if -math.pi / 4 <= heading_rad < math.pi / 4:
        heading = "east"
    elif math.pi / 4 <= heading_rad < 3 * math.pi / 4:
        heading = "north"
    elif -3 * math.pi / 4 <= heading_rad < -math.pi / 4:
        heading = "south"
    else:
        heading = "west"

    return (x, y), heading


def ir_values_to_walls(sensor_values, ir_threshold):
    return {
        "left": sensor_values["IRL"] < ir_threshold,
        "right": sensor_values["IRD"] < ir_threshold,
        "front": sensor_values["IRF"] < ir_threshold,
        "back": False
    }

def update_maze(maze, x, y, walls, heading):
    if heading == "north":
        if walls['left']:
            maze[x][y-1] = -1
        if walls['right']:
            maze[x][y+1] = -1
        if walls['front']:
            maze[x-1][y] = -1
    elif heading == "south":
        if walls['left']:
            maze[x][y+1] = -1
        if walls['right']:
            maze[x][y-1] = -1
        if walls['front']:
            maze[x+1][y] = -1
    elif heading == "east":
        if walls['left']:
            maze[x-1][y] = -1
        if walls['right']:
            maze[x+1][y] = -1
        if walls['front']:
            maze[x][y+1] = -1
    elif heading == "west":
        if walls['left']:
            maze[x+1][y] = -1
        if walls['right']:
            maze[x-1][y] = -1
        if walls['front']:
            maze[x][y-1] = -1

def process_sensor_data(sensor_data, current_position, heading):
    sensor_values = sensor_data.split(',')
    sensor_dict = {
        "IRL": int(sensor_values[1]),
        "IRD": int(sensor_values[3]),
        "IRF": int(sensor_values[5]),
        "SR1": int(sensor_values[7]),
        "SR2": int(sensor_values[9]),
        "wheelCountRight": int(sensor_values[11]),
        "wheelCountLeft": int(sensor_values[13])
    }
    wheel_radius = 0.025  # Cambiar según el radio real de la rueda en metros
    wheel_distance = 0.15  # Cambiar según la distancia real entre las ruedas en metros
    wheel_ticks = 20  # Cambiar según el número real de ticks por revolución de la rueda
    current_position, heading = update_position_and_orientation(sensor_dict, current_position, heading, wheel_radius, wheel_distance, wheel_ticks)
    return sensor_dict, current_position, heading


def read_and_update_maze(data, maze, current_position, heading, IR_THRESHOLD, goal_area):
    sensor_dict, current_position, heading = process_sensor_data(data, current_position, heading)
    walls = ir_values_to_walls(sensor_dict, IR_THRESHOLD)
    update_maze(maze, current_position[0], current_position[1], walls, heading)
    return maze, current_position, goal_area, heading


def print_maze(maze):
    for row in maze:
        for cell in row:
            if cell == -1:
                print("█", end=" ")
            elif cell == 1000000:
                print(" ", end=" ")
            else:
                print(".", end=" ")
        print()

    
def send_movements_to_robot(movements, ser):
    for movement in movements:
        ser.write(movement.encode())
        time.sleep(0.5) 

def get_goal_area(maze):
    rows, cols = len(maze), len(maze[0])
    goal_area = [
        ((rows - 2) // 2, (cols - 2) // 2),
        ((rows - 2) // 2, (cols + 2) // 2),
        ((rows + 2) // 2, (cols - 2) // 2),
        ((rows + 2) // 2, (cols + 2) // 2)
    ]
    return goal_area

def all_nodes_visited(maze, visited):
    for row in range(len(maze)):
        for col in range(len(maze[0])):
            if maze[row][col] != -1 and (row, col) not in visited:
                return False
    return True 

def get_start_position(maze, corner):
    if corner == 'top_left':
        return (0, 0)
    elif corner == 'top_right':
        return (0, len(maze[0]) - 1)
    elif corner == 'bottom_left':
        return (len(maze) - 1, 0)
    elif corner == 'bottom_right':
        return (len(maze) - 1, len(maze[0]) - 1)


def main():
    ser = serial.Serial('COM3', 115200)
    phase = 1
    maze = init_maze(12, 7)

    goal_area = get_goal_area(maze)
    start = get_start_position(maze, 'top_left')  # Reemplazar

    current_position = start
    heading = 'north' 
    IR_THRESHOLD = 500  # Ajustar según la detección de obstáculos de los sensores IR
    visited = set()
    visited.add(start) #// Checkcheckcheck

    while True:
        if ser.in_waiting > 0:
            line = ser.readline().decode().strip()
            data = line.split(",")
            # Leer datos del sensor y actualizar la matriz del laberinto
            maze, current_position, goal_area, heading = read_and_update_maze(data, maze, current_position, heading, IR_THRESHOLD, goal_area)
            visited.add(current_position)
            # Verificar si el robot ha llegado a la meta
            if current_position in goal_area:
                if all_nodes_visited(maze, visited) and phase == 1:
                    time.sleep(15)
                    # Cambia a la fase 2 (seguimiento de la ruta óptima)
                    phase = 2
                    start = current_position
                    # Calcula la ruta óptima y envía un comando al robot para indicar el inicio de la fase 2
                    movements = flood_fill(maze, start, goal_area)
                    print("Movimientos calculados:", movements)
                    send_phase_change_to_robot(ser)

                else:
                    # En la fase 2, envía los movimientos calculados al robot
                    send_movements_to_robot(movements, ser)
                    break
            # Mostrar la matriz del laberinto en tiempo real
            print_maze(maze)

if __name__ == '__main__':
    main()
