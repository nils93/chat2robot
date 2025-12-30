from ROS_cmd_vel_publisher import*

from langchain_core.tools   import tool #damit einfache Tools erstellen, auf welche das LLM zugreifen kann

@tool
def rechner(a: int, b: int, operation: str) -> int:
    """Führt eine mathematische Operation durch.
    
    Args:
        a: Erste Zahl
        b: Zweite Zahl
        operation: Die Operation ('add', 'subtract', 'multiply', 'divide')
    """
    if operation == "add":
        return a + b
    elif operation == "subtract":
        return a - b
    elif operation == "multiply":
        return a * b
    elif operation == "divide":
        return a / b if b != 0 else "Division durch Null!"
    else:
        return "Unbekannte Operation"
    

@tool   #wandelt normale Pyhton FUnktion in Tool um, welches vom LLM-Agent verwendet werden kann
def ROS_cmd_vel(x: int, y: int, operation: str)->int:
    """Wandelt die x,y Koordinaten in ein bestimmtes Datenformat um und published es auf ein ROS-Topic. 
    
    Args:
        x: X-Koordinate des Roboters
        y: Y-Koordinate des Roboters
        operation: Die Operation ('fahre_zu_den_Koordinaten')
    """
    if operation == "fahre_zu_den_Koordinaten":
        # print("Die x-Koordiante ist: ", x)
        #send_velocity(x,y)  #schickt die Koordinaten an den Roboter
        return "ROS_cmd_vel_TOOL: Koordinaten an den Roboter geschickt!\n"

    else:
        return "Unbekannte Operation"


# Weiteres Vorgehen:
# ->Pose als "/goal_pose" an das Topic Nav2 schicken (dafür nav 2 und turtlebot3 gazebo installieren)
# ->Map reinladen und schauen ob simulation funktioniert
# ->Launchfile schreiben, in dem die trutlebot in simulation gespawnt und die nav2_navigation gestartet wird
# ->Dann Simulation starten
# ->Dann code bzw funktion schreiben, welche vom LLM-Tool cmd_vel:publisher aufgerufen wird und die Koordinaten als goal pose published