""" The wrapper looks for declareSignals(), init() and step(). These functions MUST be present. All other code on the script is also run. Additional commands, snippets or scripts can be run manually from the .cpp main file"""

def declareSignals():
    """Mandatory function, can take custom kwargs, must output list of dictionaries containing requested signals information. Data type can be DOB, VEC, AIR or SCA (double valued vector, vector valued vector, airfoil valued vector and scalar)"""
    return [
        {'name': 'input_signal', 'dType': 'DOB', 'sType': 'IN'},
        {'name': 'output_signal', 'dType': 'DOB', 'sType': 'OUT'}
    ]

def init(inputs, **kwargs):
    """Mandatory function. Must take input dictionary containing data signals. Can output output dictionary with signals in the same way. May also take custom kwargs"""
    outputs = {}
    return outputs

def step(inputs, t, dt, **kwargs):
    """Mandatory function. Must take doubles t and dt as arguments. Must also take input dictionary and return output dictionary like init(). Can also take kwargs"""
    outputs = {}
    return outputs

