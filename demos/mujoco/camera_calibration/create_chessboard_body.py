def create_chessboard(
    destination_path, file_name="chessboard", chessboard_shape=(10, 8), square_size=0.02
):
    assert len(chessboard_shape) == 2, ValueError(
        f"Expected 2 dimensional board shape, got {len(chessboard_shape)}"
    )

    pos = "0.5 0.0 0.45"
    colors = ["0 0 0 1", "1 1 1 1"]  # black'n white

    with open(destination_path + file_name + ".xml", "w") as f:
        print(f'<mujoco model="{file_name}">', file=f)
        print(f'<size nconmax="2000" njmax="2000" />', file=f)
        print(f"\t<worldbody>", file=f)
        print(f'\t\t<body name="chessboard" pos="{pos}" quat="0 0 0 0">', file=f)
        for ii in range(chessboard_shape[0]):
            for jj in range(chessboard_shape[1]):
                print(
                    f'\t\t\t<geom pos="{str(square_size * ii)} {str(square_size * jj)} 0" rgba="{colors[(ii + jj) % 2]}" size=".01 .01 .001" type="box" />',
                    file=f,
                )

        print(f'\t\t\t<joint axis="0 1 0" pos="0 0 0.01" type="free" />', file=f)
        print(f"\t\t</body>", file=f)
        print(f"\t</worldbody>", file=f)
        print(f"</mujoco>", file=f)

    f.close()


if __name__ == "__main__":
    path = "/home/goat/Documents/Master/SimulationFramework2/simulation/envs/mujoco/assets/"
    create_chessboard(path)
