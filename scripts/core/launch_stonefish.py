import subprocess
import os
import time

def kill_existing_stonefish_processes():
    try:
        # Busca qualsevol proces que s'estigui executant utilitzant StonefishRLTest
        output = subprocess.check_output(["pgrep", "-f", "stonefish_rl/build/StonefishRLTest"], text=True)
        pids = output.strip().split("\n")
        for pid in pids:
            # Per cada PID trobat, mostra un missatge i mata el proces
            subprocess.run(["kill", "-9", pid])
        # Esperar per si no ha acabat d'eliminar tot els rescursos
        time.sleep(1.0) 
    except subprocess.CalledProcessError:
        pass  # No hi ha processos antics a matar

def launch_stonefish_simulator(scene_relative_path):
    """
    Executa el simulador de Stonefish amb la escena concreta.
    scene_relative_path: ruta relativa desde l'arrel del projecte.
    """

    # Assefurar-se que no hi ha processos antics de Stonefish executant-se
    kill_existing_stonefish_processes()

    # Ruta al directorio base del proyecto
    project_root = os.path.abspath(os.path.join(os.path.dirname(__file__), '../../'))

    # Ruta al ejecutable de Stonefish
    stonefish_exe = os.path.join(project_root, "build", "StonefishRLTest")

    # Ruta completa a la escena
    scene_path = os.path.join(project_root, scene_relative_path)
    resources_path = "/" + os.path.join(project_root, "Resources")
    os.environ["RESOURCES_DIR"] = resources_path
    print(f"[INFO] RESOURCES_DIR set to: {resources_path}")
    
    # Executa l'escena
    print(f"[INFO] Executing Stonefish with the scene: {scene_path}")
    stonefish_proc = subprocess.Popen([stonefish_exe, scene_path])
