CC      = gcc
CFLAGS  = -std=c99 -Wall -Wextra -Wpedantic -I. -I../Core/Inc

# Add all test binaries here
TESTS   = test_modbus_frame

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
	$(CC) $(CFLAGS) $^ -o $@

clean:
	rm -f $(TESTS)
