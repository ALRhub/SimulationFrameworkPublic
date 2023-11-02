import itertools


class MjPhyParams:
    """
    Stores and creates parameter grids for Mujoco Physics Tests
    """

    solimp_grid = [None]

    solref_grid = list(
        itertools.product([0.001, 0.01, 0.02, 0.1, 1, 0.5], [0.01, 0.1, 0.5, 1, 2, 5])
    )

    @classmethod
    def get(cls, solimp: bool = False, solref: bool = False) -> list:
        """
        construct a big grid depending on the parameter combinations
        Args:
            solimp: bool indicating if solimp grid should be included
            solref: bool indicating if solref grid should be included

        Returns:
            all combinations of the resulting parameter grid
        """
        solimp_params = [None]
        solref_params = [None]

        if solimp:
            solimp_params = cls.solimp_grid

        if solref:
            solref_params = cls.solref_grid

        full_grid = list(itertools.product(solimp_params, solref_params))
        return full_grid
