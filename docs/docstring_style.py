"""
This is a module-level docstring that provides an overview of how to use the modules in this project.
"""

import numpy as np


def my_function(param1, param2):
    """
    This is a function that performs some operation.

    Parameters
    ----------
    param1 : int
        The first parameter of the function.
    param2 : str
        The second parameter of the function.

    Returns
    -------
    bool
        The result of the operation.

    Raises
    ------
    ValueError
        If param1 is negative.
    TypeError
        If param2 is not a string.
    """
    if param1 < 0:
        raise ValueError("param1 cannot be negative")

    if not isinstance(param2, str):
        raise TypeError("param2 must be a string")

    # Perform some operation
    result = param1 > len(param2)

    return result


class MyClass:
    """
    This is a class that represents an example class.

    Attributes
    ----------
    attr1 : int
        The first attribute of the class.
    attr2 : str
        The second attribute of the class.
    """

    def __init__(self, attr1, attr2):
        """
        Initializes a MyClass object.

        Parameters
        ----------
        attr1 : int
            The first attribute of the class.
        attr2 : str
            The second attribute of the class.
        """
        self.attr1 = attr1
        self.attr2 = attr2

    def my_method(self, param):
        """
        This is a method of MyClass.

        Parameters
        ----------
        param : list
            A list parameter.

        Returns
        -------
        np.ndarray
            An array of the input list.

        Notes
        -----
        This method performs the following steps:
        1. Converts the list to a numpy array.
        2. Returns the numpy array.
        """
        np_array = np.array(param)
        return np_array
