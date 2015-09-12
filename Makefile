.PHONY: clean All

All:
	@echo "----------Building project:[ macRat - Debug ]----------"
	@"$(MAKE)" -f  "macRat.mk"
clean:
	@echo "----------Cleaning project:[ macRat - Debug ]----------"
	@"$(MAKE)" -f  "macRat.mk" clean
