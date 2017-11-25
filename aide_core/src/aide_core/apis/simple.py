"""
Some example API.
"""

def fancify_string(string):
    """
    
    :param string: String to be fancified.
    :type string: str
    :rtype: str
    """
    
    return "~^-_*~ {} ~*-_^~".format(string)


def simple_add(one, other):
    return one + other + 3
