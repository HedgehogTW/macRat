.PHONY: clean All

All:
	@echo "----------Building project:[ Vienna - Debug ]----------"
	@$(MAKE) -f  "Vienna.mk"
clean:
	@echo "----------Cleaning project:[ Vienna - Debug ]----------"
	@$(MAKE) -f  "Vienna.mk" clean
