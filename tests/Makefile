CC      = gcc
CFLAGS  = -std=c99 -Wall -Wextra -Wpedantic -I. -I../Core/Inc -I../Core/Lib
LDFLAGS = -lm

# Add all test binaries here
TESTS   = test_modbus_frame test_pid test_encoder test_safety test_scurve test_input_shaper test_kalman

.PHONY: test clean

test: $(TESTS)
	@echo ""
	@for t in $(TESTS); do \
		echo "--- $$t ---"; \
		./$$t || exit 1; \
	done
	@echo ""
	@echo "All tests passed."

test_modbus_frame: test_modbus_frame.c ../Core/Src/modbus_frame.c
	$(CC) $(CFLAGS) $^ -o $@ $(LDFLAGS)

test_pid: test_pid.c ../Core/Lib/pid.c
	$(CC) $(CFLAGS) $^ -o $@ $(LDFLAGS)

test_encoder: test_encoder.c ../Core/Drivers/encoder.c
	$(CC) $(CFLAGS) -DENCODER_PHASE_INVERTED=0 -I../Core/Drivers $^ -o $@ $(LDFLAGS)

test_safety: test_safety.c ../Core/App/safety.c
	$(CC) $(CFLAGS) -include mock_hal.h -I../Core/App $^ -o $@ $(LDFLAGS)

clean:
	rm -f $(TESTS)

test_scurve: test_scurve.c ../Core/Lib/scurve.c
	$(CC) $(CFLAGS) $^ -o $@ $(LDFLAGS)



test_input_shaper: test_input_shaper.c ../Core/Lib/input_shaper.c
	$(CC) $(CFLAGS) $^ -o $@ $(LDFLAGS)

test_kalman: test_kalman.c ../Core/Lib/kalman_lib.c
	$(CC) $(CFLAGS) $^ -o $@ $(LDFLAGS)
