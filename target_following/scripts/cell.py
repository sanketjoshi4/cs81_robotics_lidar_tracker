# Cell class to represent an x,y coordinate pair in a grid by Josephine Nguyen
# With magic methods so that 2 Cell objects with the same coordinates are considered as "the same" object

class Cell:
    def __init__(self, coords):
        """
        Stores only [x,y] coords/indexes in grid
        @param coords: list of form [x,y] indicating cell index
        """
        self.coords = coords # expect [x,y] list

    def __hash__(self):
        """
        To make checks like 'if c in dict' work
        """
        return hash((self.coords[0], self.coords[1]))

    def __eq__(self, other):
        """
        To make checks like 'if c in dict' and 'if c == c1' work
        @param other: another Cell object to compare to
        """
        return self.coords[0] == other.coords[0] and self.coords[1] == other.coords[1]

    def __ne__(self, other):
        """
        To make checks like 'if c in dict' and 'if c == c1' work
        @param other: another Cell object to compare to
        """
        return self.coords[0] != other.coords[0] or self.coords[1] != other.coords[1]

    def __str__(self):
        """
        For debug prints
        """
        return str(self.coords[0]) + "," + str(self.coords[1])
