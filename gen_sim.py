import matplotlib.pyplot as plt
import matplotlib.animation as animation
import numpy as np
import json
import moviepy.editor as mp
from matplotlib.patches import Rectangle, Circle


PROJECT_PATH = "/home/hpcnc/bus_opt"
data_path = "/output/"
files = ["tc_25_12_60_6.json"]

def create_bus_terminal_animation(data):
    """
    Create an animated visualization of bus terminals using optimization results
    
    Parameters:
    data: dict containing optimization results with assignments, departures, and arrivals
    """
    
    # Extract data
    buses = data["sets"]["buses"]
    capacity = data["meta"]["capacity"]
    tau = data["meta"]["tau"]
    w_max = data["meta"]["w_max"]
    T_end = data["meta"]["T_end"]
    
    initial_positions = {pos["bus"]: pos["terminal"] for pos in data["initial_positions"]}
    assignments = data["assignments"]
    departures = data["departures"]
    arrivals_cei = {arr["p"]: arr["arr"] for arr in data["arrivals"]["CEI"]}
    arrivals_t2 = {arr["p"]: arr["arr"] for arr in data["arrivals"]["T2"]}


    # Setup figure
    fig, ax = plt.subplots(figsize=(18, 12))
    ax.set_xlim(-2, 14)
    ax.set_ylim(-4, 8)
    ax.set_aspect('equal')
    ax.axis('off')
    ax.set_title('Bus Terminal Optimization Results Animation', fontsize=16, fontweight='bold', pad=20)
    
    # Terminal positions
    terminal_positions = {
        'CEI': (2, 2),
        'T2': (10, 2)
    }
    
    # Initialize bus states
    bus_states = {}
    for bus_id in buses:
        initial_terminal = initial_positions[bus_id]
        x_pos = terminal_positions[initial_terminal][0]
        # y_offset = (bus_id % max(n_cei, n_t2)) * 1.5 - 0.75  # Spread buses vertically

        # if x_pos > 6:
        #     y_offset -= 0.75  # Shift down for T2 side

        bus_states[bus_id] = {
            'x': x_pos,
            'y': terminal_positions[initial_terminal][1] + 2.5, #- 1.0 + y_offset,
            'current_terminal': initial_terminal,
            'target_terminal': None,
            'passengers': [],  # List of passenger IDs
            'status': 'at_terminal',
            'travel_progress': 0,
            'departure_time': None,
            'arrival_time': None
        }
    
    # Initialize passenger states
    passenger_states = {}
    
    # Process all passengers from CEI
    for p_id, arr_time in arrivals_cei.items():
        passenger_states[p_id] = {
            'origin': 'CEI',
            'destination': 'T2',
            'arrival_time': arr_time,
            'status': 'waiting',  # waiting, on_bus, completed
            'assigned_bus': None,
            'pickup_time': None,
            'wait_time': 0
        }
    
    # Process all passengers from T2
    for p_id, arr_time in arrivals_t2.items():
        passenger_states[p_id+len(arrivals_cei)] = {
            'origin': 'T2',
            'destination': 'CEI',
            'arrival_time': arr_time,
            'status': 'waiting',
            'assigned_bus': None,
            'pickup_time': None,
            'wait_time': 0
        }
    
    # Process assignments to determine which bus picks up which passenger
    for assignment in assignments:
        p_id = assignment["p"]
        bus_id = assignment["bus"]
        pickup_time = assignment["t"]
        terminal = assignment["terminal"]
        if terminal == 'T2':
            p_id += len(arrivals_cei)  # Adjust ID for T2 passengers
        if p_id in passenger_states:
            passenger_states[p_id]['assigned_bus'] = bus_id
            passenger_states[p_id]['pickup_time'] = pickup_time
        
    # Process departures
    departure_schedule = {}
    for departure in departures:
        time = departure["t"]
        bus_id = departure["bus"]
        terminal = departure["terminal"]
        
        if time not in departure_schedule:
            departure_schedule[time] = []
        departure_schedule[time].append({'bus': bus_id, 'terminal': terminal})
    
    # Colors for buses
    bus_colors = plt.cm.Set3(np.linspace(0, 1, len(buses)))
    
    def get_waiting_passengers(terminal, current_time):
        """Get passengers currently waiting at a terminal"""
        waiting = []
        for p_id, state in passenger_states.items():
            if (state['origin'] == terminal and 
                state['arrival_time'] <= current_time and 
                state['status'] == 'waiting'):
                waiting.append(p_id)
        return waiting

    def get_passengers_on_bus(bus_id):
        """Get passengers currently on a bus"""
        on_bus = []
        for p_id, state in passenger_states.items():
            if state['assigned_bus'] == bus_id and state['status'] == 'on_bus':
                on_bus.append(p_id)
        return on_bus

    def draw_terminal(pos, name, waiting_passengers, current_time):
        """Draw terminal building and passenger queue"""
        x, y = pos
        
        # Terminal building
        terminal = Rectangle((x-0.8, y-4), 1.6, 8, 
                            facecolor='lightsteelblue', edgecolor='navy', linewidth=3)
        ax.add_patch(terminal)
        
        # Terminal name
        ax.text(x, y+4.25, name, ha='center', va='center', fontsize=16, fontweight='bold')
        
        # Passenger count
        passenger_count = len(waiting_passengers)
        ax.text(x, y-3.5, f'Waiting: {passenger_count}', 
                ha='center', va='center', fontsize=12, fontweight='bold',
                bbox=dict(boxstyle="round,pad=0.3", facecolor='lightyellow'))
        
        # Draw passenger icons with IDs
        passengers_per_row = 3
        max_passengers_display = 27  # Max passengers to display
        if x > 6: # T2 side
            for i, p_id in enumerate(waiting_passengers[:max_passengers_display]):  # Show max 24 passengers
                row = i // passengers_per_row
                col = i % passengers_per_row
                px = x + 2 + col * 0.7
                py = y + 3 - row * 0.6
                
                # Passenger circle
                passenger_circle = Circle((px, py), 0.12, 
                                        facecolor='orange', edgecolor='darkorange', linewidth=2)
                ax.add_patch(passenger_circle)
                
                # Passenger ID
                ax.text(px, py, str(p_id-len(arrivals_cei)), ha='center', va='center', 
                    fontsize=12, fontweight='bold', color='white')
                
                # Show wait time
                wait_time = current_time - passenger_states[p_id]['arrival_time']
                ax.text(px, py-0.25, f'{wait_time}min', ha='center', va='center', 
                    fontsize=8, bbox=dict(boxstyle="round,pad=0.1", facecolor='white', alpha=0.8))
            
            if passenger_count > 24:
                ax.text(x+3, y-2.5, f'+{passenger_count-24} more passengers', 
                    ha='center', va='center', fontsize=10)
        else:  # CEI side
            for i, p_id in enumerate(waiting_passengers[:max_passengers_display]):  # Show max 24 passengers
                row = i // passengers_per_row
                col = i % passengers_per_row
                px = x - 2 - col * 0.7
                py = y + 3 - row * 0.6
                
                # Passenger circle
                passenger_circle = Circle((px, py), 0.12, 
                                        facecolor='orange', edgecolor='darkorange', linewidth=2)
                ax.add_patch(passenger_circle)
                
                # Passenger ID
                ax.text(px, py, str(p_id), ha='center', va='center', 
                    fontsize=9, fontweight='bold', color='white')
                
                # Show wait time
                wait_time = current_time - passenger_states[p_id]['arrival_time']
                ax.text(px, py-0.25, f'{wait_time}min', ha='center', va='center', 
                    fontsize=8, bbox=dict(boxstyle="round,pad=0.1", facecolor='white', alpha=0.8))
            
            if passenger_count > 24:
                ax.text(x-2, y-2.5, f'+{passenger_count-24} more passengers', 
                    ha='center', va='center', fontsize=10)
                
    def draw_bus(bus_id, state, color, current_time):
        """Draw a bus with its current state and passengers"""
        x, y = state['x'], state['y']
        
        # Bus body (larger to show more detail)
        bus_rect = Rectangle((x-0.5, y-(0.75*(bus_id-1))), 1.0, 0.5, 
                            facecolor=color, edgecolor='black', linewidth=2)
        ax.add_patch(bus_rect)
        
        # Bus ID
        ax.text(x, y-(0.75*(bus_id-1))+0.25, f'Bus {bus_id}', ha='center', va='center', 
                fontsize=10, fontweight='bold')
        
        # Get passengers on this bus
        passengers_on_bus = get_passengers_on_bus(bus_id)
        
        # Passenger count and capacity
        ax.text(x, y-(0.75*(bus_id-1))+0.5, f'Passengers: {len(passengers_on_bus)}/{capacity}', 
                ha='center', va='center', fontsize=9, fontweight='bold',
                bbox=dict(boxstyle="round,pad=0.2", facecolor='white', alpha=0.9))

    def update_simulation(current_time):
        """Update simulation state based on current time"""
        
        # Handle departures
        if current_time in departure_schedule:
            for departure in departure_schedule[current_time]:
                bus_id = departure['bus']
                terminal = departure['terminal']
                
                if bus_states[bus_id]['status'] == 'at_terminal':
                    # Pick up assigned passengers
                    for full_id, p_state in passenger_states.items():
                        if (p_state['assigned_bus'] == bus_id and 
                            p_state['pickup_time'] == current_time and
                            p_state['status'] == 'waiting' and
                            p_state['origin'] == terminal):
                            p_state['status'] = 'on_bus'
                            bus_states[bus_id]['passengers'].append(full_id)
                            if p_state['origin'] == 'T2':
                                p_id = full_id - len(arrivals_cei)
                    
                    # Start traveling
                    bus_states[bus_id]['status'] = 'traveling'
                    bus_states[bus_id]['target_terminal'] = 'T2' if terminal == 'CEI' else 'CEI'
                    bus_states[bus_id]['travel_progress'] = 0
                    bus_states[bus_id]['departure_time'] = current_time
                    bus_states[bus_id]['arrival_time'] = current_time + tau
        
        # Update traveling buses
        for bus_id, state in bus_states.items():
            if state['status'] == 'traveling':
                progress = (current_time - state['departure_time']) / tau
                
                if progress >= 1.0:
                    # Arrived at destination
                    state['status'] = 'at_terminal'
                    state['current_terminal'] = state['target_terminal']
                    state['x'] = terminal_positions[state['current_terminal']][0]
                    
                    # Passengers disembark and complete their journey
                    for p_id in state['passengers']:
                        passenger_states[p_id]['status'] = 'completed'
                    state['passengers'] = []
                    state['target_terminal'] = None
                    
                else:
                    # Update position
                    start_pos = terminal_positions[state['current_terminal']]
                    end_pos = terminal_positions[state['target_terminal']]
                    state['x'] = start_pos[0] + (end_pos[0] - start_pos[0]) * progress
    
    def animate(frame):
        ax.clear()
        ax.set_xlim(-2, 14)
        ax.set_ylim(-4, 8)
        ax.set_aspect('equal')
        ax.axis('off')
        ax.set_title('Bus Terminal Optimization Results Animation', fontsize=16, fontweight='bold', pad=20)
        
        current_time = frame // 2  # 2 frames per time unit
        
        # Update simulation
        update_simulation(current_time)
        
        # Draw terminals with waiting passengers
        cei_waiting = get_waiting_passengers('CEI', current_time)
        # print("cei_waiting",cei_waiting)
        t2_waiting = get_waiting_passengers('T2', current_time)
        # print("t2_waiting",t2_waiting)
        
        draw_terminal(terminal_positions['CEI'], 'CEI', cei_waiting, current_time)
        draw_terminal(terminal_positions['T2'], 'T2', t2_waiting, current_time)
        
        # Draw buses
        for i, bus_id in enumerate(buses):
            draw_bus(bus_id, bus_states[bus_id], bus_colors[i], current_time)
        
        # Statistics
        total_waiting = len(cei_waiting) + len(t2_waiting)
        total_on_buses = sum(len(get_passengers_on_bus(bus_id)) for bus_id in buses)
        total_completed = sum(1 for p in passenger_states.values() if p['status'] == 'completed')
        
        ax.text(6, 6.5, f'Time: {current_time} minutes', 
               ha='center', va='center', fontsize=14, fontweight='bold',
               bbox=dict(boxstyle="round,pad=0.4", facecolor='lightgray'))
        
        ax.text(6, 6.0, f'Waiting: {total_waiting} | On Buses: {total_on_buses} | Completed: {total_completed}', 
               ha='center', va='center', fontsize=12,
               bbox=dict(boxstyle="round,pad=0.3", facecolor='lightyellow'))
        
        # Objective value
        ax.text(6, 5.5, f'Objective Value: {data["objective"]} | Status: {data["status"]}', 
               ha='center', va='center', fontsize=11,
               bbox=dict(boxstyle="round,pad=0.3", facecolor='lightgreen'))
        
        # Progress bar
        progress = min(current_time / T_end, 1.0) if T_end > 0 else 0
        progress_bar = Rectangle((-1, 7), 14 * progress, 0.3, 
                               facecolor='green', alpha=0.7)
        ax.add_patch(progress_bar)
        ax.text(6, 7.5, f'Progress: {progress*100:.1f}%', ha='center', va='center', fontsize=10)
    
    # Create animation
    anim = animation.FuncAnimation(fig, animate, frames=T_end*2, interval=100, repeat=True)
    
    plt.tight_layout()
    return fig, anim


for filename in files:
    try:
        with open(f"{PROJECT_PATH}/{data_path}/{filename}", "r") as f:
            data = json.load(f)
    except Exception as e:
        print(f"error reading {filename}: {e}")
        continue

    try:
        print(f"creating animation for {filename}...")
        fig, anim = create_bus_terminal_animation(data)
        anim.save(PROJECT_PATH + f"/animation/{filename}.gif", writer='pillow', fps=10)
        print(f"saved animation...")
        clip = mp.VideoFileClip(PROJECT_PATH + f"/animation/{filename}.gif")
        print("converting to MP4...")
        clip.write_videofile(f"{filename}.mp4")
        print("===== finished ====")
    except Exception as e:
        print(f"error creating aimation for {filename} : {e}")
