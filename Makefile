CARGO:=$(shell which cargo)

all:

.PHONY: run_left
run_left:
	${CARGO} run -- --probe ${LEFT_PROBE}


.PHONY: run_right
run_right:
	${CARGO} run -- --probe ${RIGHT_PROBE}
