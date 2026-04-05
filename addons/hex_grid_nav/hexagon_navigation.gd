extends Node

# Method documentation: https://docs.google.com/document/d/1HwLlRmC2tDGbadkOEero5asVq6XmzpupeJX_pUEwqGg/edit?usp=sharing

var current_map : TileMapLayer
var astar = AStar2D.new()

# Maps Vector2i (grid) -> int (AStar ID)
var tile_to_id_map : Dictionary = {}

func are_tiles_connected(left_id: int, right_id: int) -> bool:
	return astar.are_points_connected(left_id, right_id)
	
## Initial function that binds a [TileMapLayer] to the navigation class
func set_current_map(map : TileMapLayer):
	current_map = map
	if current_map != null:
		astar.clear()
		tile_to_id_map.clear()
		add_all_point()
	print("there are " + str(astar.get_point_count()) + " points in this map")

## Adds and connects all cells in the [TileMapLayer]
func add_all_point(): 
	var all_used_cells = current_map.get_used_cells()
	
	# Add points using local (pixel) coordinates for accurate Euclidean heuristics
	for cell in all_used_cells:
		var next_id := astar.get_available_point_id()
		astar.add_point(next_id, current_map.map_to_local(cell))
		tile_to_id_map[cell] = next_id

	# Connect neighbors based on TileMap hexagon adjacency
	for cell in all_used_cells:
		var point_id = tile_to_id(cell)
		var all_possible_neighbors = current_map.get_surrounding_cells(cell)
		for neighbor in all_possible_neighbors:
			var neighbor_id = tile_to_id(neighbor)
			if neighbor_id != -1:
				# connect_points is bidirectional by default
				astar.connect_points(point_id, neighbor_id)

## Set the [member weight_scale] of all tiles in the data layer with [param layer_name] that meet [param condition] to [param new_weight]
func set_weight_of_layer(layer_name: String, condition: Variant, new_weight: float) -> void:
	var all_point_id = astar.get_point_ids()
	for id in all_point_id:
		var tile = id_to_tile(id)
		if get_cell_custom_data(tile, layer_name) == condition:
			astar.set_point_weight_scale(id, new_weight)

## Returns the global position of a cell position
func cell_to_global(cell_pos : Vector2i) -> Vector2:
	return current_map.to_global(current_map.map_to_local(cell_pos))

## Returns the cell position of a global position; returns [code]Vector2i(-999, 999)[/code] if no valid cell is found.
func global_to_cell(global_pos : Vector2) -> Vector2i:
	var local_pos := current_map.to_local(global_pos)
	var closest_point_id = astar.get_closest_point(local_pos)
	
	if closest_point_id == -1: 
		return Vector2i(-999, -999)
	return id_to_tile(closest_point_id)

## Returns the cell position that is closest to a given global position
func get_closest_cell_by_global_pos(global_pos : Vector2) -> Vector2i:
	var local_pos := current_map.to_local(global_pos)
	var closest_point_id = astar.get_closest_point(local_pos)
	return id_to_tile(closest_point_id)

func get_cell_custom_data(cell_pos: Vector2i, data_name: String):
	var data = current_map.get_cell_tile_data(cell_pos)
	if data:
		return data.get_custom_data(data_name)
	return null

## Returns an array of cell positions (Vector2i) from start to goal
func get_navi_path(start_pos : Vector2i, end_pos : Vector2i) -> Array[Vector2i]:
	var start_id = tile_to_id(start_pos)
	var goal_id = tile_to_id(end_pos)
	
	if start_id == -1 or goal_id == -1:
		return []
		
	var pixel_path = astar.get_point_path(start_id, goal_id)
	var cell_path: Array[Vector2i] = []
	for point in pixel_path:
		cell_path.append(current_map.local_to_map(point))
	
	return cell_path

## Use a tile position to get the id for AStar usage
func tile_to_id(pos: Vector2i) -> int: 
	return tile_to_id_map.get(pos, -1)

## Use an AStar point ID to get the point's tile position
func id_to_tile(id: int) -> Vector2i:
	if astar.has_point(id):
		var pixel_pos = astar.get_point_position(id)
		return current_map.local_to_map(pixel_pos)
	return Vector2i(-1, -1)

func get_distance(pos1: Vector2i, pos2: Vector2i) -> int:
	var path = get_navi_path(pos1, pos2)
	return max(0, path.size() - 1)

## Returns all neighbor cells within [param range]. Uses a depth first search.
func get_all_neighbors_in_range(start_pos: Vector2i, range: int, max_weight: float = 1.0) -> Array[Vector2i]:
	var all_neighbors_id : Array[int] = []
	var starting_cell_id = tile_to_id(start_pos)
	if starting_cell_id == -1: return []
	
	_dfs(range, starting_cell_id, starting_cell_id, all_neighbors_id, max_weight)
	
	var all_neighbors_pos = all_neighbors_id.map(id_to_tile)
	var answer: Array[Vector2i]
	answer.assign(all_neighbors_pos)
	return answer

func _dfs(k : int, node_id : int, parent_id : int, solution_arr : Array, max_weight: float):
	if k < 0 or node_id == -1:
		return
	if astar.get_point_weight_scale(node_id) > max_weight:
		return
	if !solution_arr.has(node_id):
		solution_arr.append(node_id)
		
	var cell_pos: Vector2i = id_to_tile(node_id)
	for neighbor_pos in current_map.get_surrounding_cells(cell_pos):
		var neighbor_id = tile_to_id(neighbor_pos)
		if neighbor_id != parent_id:
			_dfs(k-1, neighbor_id, node_id, solution_arr, max_weight)

## Return all tiles with [param value] in the custom data value with name [param custom_data_name]
func get_all_tile_with_layer(custom_data_name: String, value: Variant) -> Array[Vector2i]:
	var valid_tiles: Array[Vector2i] = []
	for id in astar.get_point_ids():
		var tile = id_to_tile(id)
		if get_cell_custom_data(tile, custom_data_name) == value:
			valid_tiles.append(tile)
	return valid_tiles

## Returns a random tile position (Vector2i) from the registered tile set
func get_random_tile_pos() -> Vector2i:
	var all_ids = astar.get_point_ids()
	if all_ids.is_empty(): return Vector2i(-1, -1)
	return id_to_tile(all_ids[randi() % all_ids.size()])