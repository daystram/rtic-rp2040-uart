CARGO:=$(shell which cargo)

all:

.PHONY: run_master
run_master:
	${CARGO} run -- --probe ${MASTER_PROBE}


.PHONY: run_slave
run_slave:
	${CARGO} run -- --probe ${SLAVE_PROBE}
