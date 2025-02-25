extends TileMapLayer

const MAIN_ATLAS_ID = 0
const BLUE_CELL := Vector2i(0, 0)
const WHITE_CELL := Vector2i(2, 0)
const RED_CELL := Vector2i(1, 0)
	
func set_cell_to_variant(id : int, cell : Vector2i):
	var cell_variant
	match id:
		0: cell_variant = WHITE_CELL
		1: cell_variant = RED_CELL
		2: cell_variant = BLUE_CELL
	set_cell(cell, MAIN_ATLAS_ID, cell_variant)

func clear_cells():
	for pos in get_used_cells():
		set_cell(pos, MAIN_ATLAS_ID, WHITE_CELL)
