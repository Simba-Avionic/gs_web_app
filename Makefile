PYTHON ?= python3

UNIT_TESTS := \
	tests/unit/test_mavlink_bridge.py \
	tests/unit/test_xml_to_json.py

TEST_TARGETS := \
	tests-unit

ROS_SIM := tests/ros/dynamic_publisher_sim.py

.PHONY: tests test tests-unit ros-sim
tests:
	@set -e; \
	for test_target in $(TEST_TARGETS); do \
		echo "==> $$test_target"; \
		$(MAKE) --no-print-directory "$$test_target"; \
	done

test: tests

tests-unit:
	@set -e; \
	for test_file in $(UNIT_TESTS); do \
		echo "==> $$test_file"; \
		$(PYTHON) -m unittest "$$test_file"; \
	done

ros-sim:
	$(PYTHON) $(ROS_SIM)
