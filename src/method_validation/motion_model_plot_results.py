import matplotlib.pyplot as plt
import csv
import math
import subprocess
import os
import shutil  # Library to move files

# --- CONFIGURATION ---
# Set the name of the output folder inside method_validation
OUTPUT_DIR = "results" 
# ---------------------

def run_pipeline():
    # Ensure results folder exists
    if not os.path.exists(OUTPUT_DIR):
        os.makedirs(OUTPUT_DIR)
        print(f"📁 Folder '{OUTPUT_DIR}' created.")

    print("--- 1. Compiling C++ Code ---")
    
    compile_cmd = [
        "g++", 
        "-I..", 
        "simulation/motion_model_validation.cpp", 
        "-o", "simulation/validation_test"
    ]
    
    compile_process = subprocess.run(compile_cmd, capture_output=True, text=True)
    
    if compile_process.returncode != 0:
        print("❌ Compilation Failed!")
        print("Error message:\n", compile_process.stderr)
        return False
    else:
        print("✅ Compilation Successful.")

    print("\n--- 2. Running C++ Simulation ---")
    
    # The C++ executable will generate 'motion_model_data.csv' inside 'simulation' folder
    run_cmd = ["./validation_test"]
    run_process = subprocess.run(run_cmd, cwd="simulation")
    
    if run_process.returncode != 0:
        print("❌ Runtime Error!")
        return False
    
    # --- MOVE CSV FILE ---
    source_csv = "simulation/motion_model_data.csv"
    dest_csv = os.path.join(OUTPUT_DIR, "motion_model_data.csv")
    
    # Move (or replace) the file to the results folder
    if os.path.exists(source_csv):
        shutil.move(source_csv, dest_csv)
        print(f"✅ Simulation finished. CSV moved to '{dest_csv}'.")
    else:
        print("❌ Error: CSV not found in simulation folder.")
        return False
    
    return True

def plot_results():
    print("\n--- 3. Generating Plot ---")
    x = []
    y = []
    theta = []

    # Path to CSV inside the results folder
    csv_path = os.path.join(OUTPUT_DIR, "motion_model_data.csv")

    try:
        with open(csv_path, 'r') as csvfile:
            plots = csv.reader(csvfile, delimiter=',')
            next(plots) # Skip header
            for row in plots:
                x.append(float(row[0]))
                y.append(float(row[1]))
                theta.append(float(row[2]))
    except FileNotFoundError:
        print(f"Error: File '{csv_path}' not found.")
        exit()

    # Creating the plot
    plt.figure(figsize=(10, 6))
    plt.scatter(x, y, label='Particles (Hypotheses)', color='blue', alpha=0.5, s=10)
    plt.plot(0, 0, 'go', markersize=10, label='Start (0,0)') 
    plt.plot(2.0, 0, 'rx', markersize=10, label='Theoretical Target (2,0)')

    for i in range(0, len(x), 5):
        plt.arrow(x[i], y[i], 0.1 * math.cos(theta[i]), 0.1 * math.sin(theta[i]), 
                  head_width=0.05, head_length=0.05, fc='gray', ec='gray', alpha=0.3)

    plt.title('Motion Model Validation (FastSLAM)')
    plt.xlabel('X (m)')
    plt.ylabel('Y (m)')
    plt.axis('equal')
    plt.grid(True)
    plt.legend()

    # SAVE PNG IN RESULTS FOLDER
    png_path = os.path.join(OUTPUT_DIR, "motion_model_plot.png")
    plt.savefig(png_path)
    print(f"✅ Plot saved as '{png_path}'")
    plt.show()

if __name__ == "__main__":
    if run_pipeline():
        plot_results()