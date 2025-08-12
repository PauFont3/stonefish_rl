import subprocess
import os
import time

def kill_existing_stonefish_processes():
    try:
        # # Find any process currently running StonefishRLTest
        output = subprocess.check_output(["pgrep", "-f", "stonefish_rl/build/StonefishRLTest"], text=True)
        pids = output.strip().split("\n")
        for pid in pids:
            # For each PID found, kills the process
            #print(f"[INFO] Killing old process from StonefishRLTest with PID {pid}")
            subprocess.run(["kill", "-9", pid])
        # Wait in case not all resources have been released
        time.sleep(1.0) 
    except subprocess.CalledProcessError:
        pass  # There are no prvious processes to kill

def launch_stonefish_simulator(scene_relative_path):
    """
    Launch the Stonefish simulator with the specified scene.
    scene_relative_path: path relative to the project root.
    """

    # Make sure that there are no old Stonefish processes running
    kill_existing_stonefish_processes()

    # Path to the project root directory
    project_root = os.path.abspath(os.path.join(os.path.dirname(__file__), '../../'))

    # Path to the Stonefish executable
    stonefish_exe = os.path.join(project_root, "build", "StonefishRLTest")

    # Full path to the scene
    scene_path = os.path.join(project_root, scene_relative_path)

    # Run the scene
    print(f"[INFO] Executing Stonefish with the scene: {scene_path}")
    stonefish_proc = subprocess.Popen([stonefish_exe, scene_path])
