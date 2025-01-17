@tool
extends EditorPlugin

const HEX_GRID_NAV = "HexNavi"
#The singleton (global) class name of the navigation system is called HexNavi.
#Call this class whereever you need the navigation system.

func _enable_plugin():
	add_autoload_singleton(HEX_GRID_NAV, "res://addons/hex_grid_nav/hexagon_navigation.gd")

func _disable_plugin():
	remove_autoload_singleton(HEX_GRID_NAV)

func _enter_tree():
	# Initialization of the plugin goes here.
	pass

func _exit_tree():
	# Clean-up of the plugin goes here.
	pass
