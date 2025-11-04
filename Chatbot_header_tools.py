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