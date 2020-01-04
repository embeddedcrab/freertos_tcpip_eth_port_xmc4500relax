

# Include make sources
-include sources.mk
-include objects.mk


# Asm source file compilation
$(BUILD_DIR)/%.o: %.s
	mkdir -p $(dir $@)
	@echo 'Invoking assembler for target: $@'
	$(CC) -x assembler-with-cpp $(C_FLAGS) $(ASFLAGS) $< -o $@
	@echo 'Done generating target: $@'
	@echo '.'


# C source file compilation
$(BUILD_DIR)/%.o: %.c
	mkdir -p $(dir $@)
	@echo 'Invoking GCC Compiler for target: $@'
	$(CC) $(C_FLAGS) $(CFLAGS) $< -o $@
	@echo 'Done generating target: $@'
	@echo '.'


# CPP source file compilation
$(BUILD_DIR)/%.o: %.cpp
	mkdir -p $(dir $@)
	@echo 'Invoking G++ Compiler for target: $@'
	$(CXX) $(C_FLAGS) $(CXX_FLAGS) $< -o $@
	@echo 'Done generating target: $@'
	@echo '.'


