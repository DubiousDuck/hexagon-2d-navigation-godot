<img src="HexNaviIcon.png" alt="icon"/>

# Hexagon TileMapLayer Navigation Class
This is a Godot plugin that adds a global class that handles navigation on 2D [TileMapLayers](https://docs.godotengine.org/en/stable/classes/class_tilemaplayer.html) with Hexagon tiles.

Works for Godot version 4.3 and up.

You can install it via the AssetLibrary in the Godot editor or download [the repo](https://github.com/DubiousDuck/hexagon-navigation-godot) on GitHub.

## How to Install

- Copy the 'hex_grid_nav' folder to your 'addons' folder in your Godot project
- In Project Settings, under the "Plugin" tab, enable the checkbox next to "Hexagon Grid Navigation".
- You will now see the class "HexNavi" has been added to Global Autoload

## How to use

A detailed explanation of the functions available in the HexNavi class can be found in [this documentaion](https://docs.google.com/document/d/1HwLlRmC2tDGbadkOEero5asVq6XmzpupeJX_pUEwqGg/edit?tab=t.0#heading=h.p2jx3zil9jqy)

Here are some initial steps to set up the class properly:

- Add a TileMapLayer node to your desired scene
- Set the Tile Shape in Tile Set of the TileMapLayer to `Hexagon`
- Call `HexNavi.set_current_map()` at the beginning of your script and use the TileMapLayer you just set up as the input
- Congrats! You have successfully set up the navigation system. You can now call whichever functions in [the documentation](https://docs.google.com/document/d/1HwLlRmC2tDGbadkOEero5asVq6XmzpupeJX_pUEwqGg/edit?usp=sharing) that you see fit.

Check out the [example project](https://github.com/DubiousDuck/hexagon-2d-navigation-godot/tree/main/example-project) to see how to use the different functions of the class!

## Contact

If you have any questions, comments, and suggestions, please don't hesitate to reach out to <oscar517730@gmail.com>
