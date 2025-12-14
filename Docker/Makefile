# R2D3 Robot Development Environment Makefile
# Convenient commands for managing the R2D3 robot environment

.PHONY: help setup start stop restart build shell logs clean status
.DEFAULT_GOAL := help

# Default distribution
DISTRO ?= humble

# Auto-detect hostname
HOSTNAME ?= $(shell hostname)

# Colors
BLUE := \033[0;34m
GREEN := \033[0;32m
YELLOW := \033[1;33m
RED := \033[0;31m
NC := \033[0m

help: ## Show this help message
	@echo "$(BLUE)R2D3 Robot Development Environment$(NC)"
	@echo ""
	@echo "$(YELLOW)Usage:$(NC)"
	@echo "  make [target] [DISTRO=foxy|humble|jazzy]"
	@echo ""
	@echo "$(YELLOW)Targets:$(NC)"
	@awk 'BEGIN {FS = ":.*?## "} /^[a-zA-Z_-]+:.*?## / {printf "  $(GREEN)%-15s$(NC) %s\n", $$1, $$2}' $(MAKEFILE_LIST)
	@echo ""
	@echo "$(YELLOW)Examples:$(NC)"
	@echo "  make setup              # Setup host environment"
	@echo "  make start              # Start default (Humble) environment"
	@echo "  make start DISTRO=foxy  # Start Foxy environment"
	@echo "  make build DISTRO=jazzy # Build Jazzy image"
	@echo "  make shell              # Open shell in running container"
	@echo "  make clean              # Clean up everything"

setup: ## Setup host environment (X11, PulseAudio)
	@echo "$(BLUE)Setting up host environment...$(NC)"
	@mkdir -p /run/user/$$(id -u)/pulse 2>/dev/null || true
	@if command -v xhost >/dev/null 2>&1; then \
		xhost +local:docker >/dev/null 2>&1 || echo "$(YELLOW)Warning: Failed to setup X11 forwarding$(NC)"; \
	else \
		echo "$(YELLOW)Warning: xhost not found. Install x11-xserver-utils for GUI support.$(NC)"; \
	fi
	@if command -v pulseaudio >/dev/null 2>&1; then \
		if ! pgrep -x pulseaudio >/dev/null; then \
			echo "$(BLUE)Starting PulseAudio...$(NC)"; \
			pulseaudio --start 2>/dev/null || echo "$(YELLOW)Warning: Failed to start PulseAudio$(NC)"; \
		fi; \
	else \
		echo "$(YELLOW)Warning: PulseAudio not found. Audio may not work in containers.$(NC)"; \
	fi
	@if [ ! -f .env ]; then \
		if [ -f env.example ]; then \
			echo "$(BLUE)Creating .env file from env.example$(NC)"; \
			cp env.example .env; \
		fi; \
	fi
	@echo "$(GREEN)Host environment setup complete!$(NC)"

start: setup ## Start R2D3 environment (default: humble)
	@echo "$(BLUE)Starting R2D3 $(DISTRO) environment...$(NC)"
	@if command -v docker-compose >/dev/null 2>&1; then \
		COMPOSE_CMD="docker-compose"; \
	else \
		COMPOSE_CMD="docker compose"; \
	fi; \
	if [ "$(DISTRO)" = "humble" ]; then \
		HOSTNAME=$(HOSTNAME) $$COMPOSE_CMD --profile default up -d; \
	else \
		HOSTNAME=$(HOSTNAME) $$COMPOSE_CMD --profile $(DISTRO) up -d; \
	fi
	@echo "$(GREEN)R2D3 $(DISTRO) environment started!$(NC)"
	@echo "$(BLUE)Connect with: make shell DISTRO=$(DISTRO)$(NC)"

stop: ## Stop R2D3 environment
	@echo "$(BLUE)Stopping R2D3 $(DISTRO) environment...$(NC)"
	@if command -v docker-compose >/dev/null 2>&1; then \
		COMPOSE_CMD="docker-compose"; \
	else \
		COMPOSE_CMD="docker compose"; \
	fi; \
	if [ "$(DISTRO)" = "humble" ]; then \
		HOSTNAME=$(HOSTNAME) $$COMPOSE_CMD --profile default down; \
	else \
		HOSTNAME=$(HOSTNAME) $$COMPOSE_CMD --profile $(DISTRO) down; \
	fi
	@echo "$(GREEN)R2D3 $(DISTRO) environment stopped!$(NC)"

restart: ## Restart R2D3 environment
	@echo "$(BLUE)Restarting R2D3 $(DISTRO) environment...$(NC)"
	@$(MAKE) stop DISTRO=$(DISTRO)
	@sleep 2
	@$(MAKE) start DISTRO=$(DISTRO)

build: ## Build R2D3 images
	@echo "$(BLUE)Building R2D3 $(DISTRO) image...$(NC)"
	@if command -v docker-compose >/dev/null 2>&1; then \
		COMPOSE_CMD="docker-compose"; \
	else \
		COMPOSE_CMD="docker compose"; \
	fi; \
	if [ "$(DISTRO)" = "humble" ]; then \
		HOSTNAME=$(HOSTNAME) $$COMPOSE_CMD --profile default build --no-cache; \
	else \
		HOSTNAME=$(HOSTNAME) $$COMPOSE_CMD --profile $(DISTRO) build --no-cache; \
	fi
	@echo "$(GREEN)R2D3 $(DISTRO) image built!$(NC)"

shell: ## Open shell in running container
	@if ! docker ps --format "table {{.Names}}" | grep -q "^r2d3-$(DISTRO)$$"; then \
		echo "$(RED)Error: Container r2d3-$(DISTRO) is not running.$(NC)"; \
		echo "$(BLUE)Start it with: make start DISTRO=$(DISTRO)$(NC)"; \
		exit 1; \
	fi
	@echo "$(BLUE)Opening shell in R2D3 $(DISTRO) container...$(NC)"
	@docker exec -it r2d3-$(DISTRO) bash

logs: ## Show container logs
	@echo "$(BLUE)Showing logs for R2D3 $(DISTRO) container...$(NC)"
	@docker logs -f r2d3-$(DISTRO) 2>/dev/null || { \
		echo "$(RED)Error: Container r2d3-$(DISTRO) not found.$(NC)"; \
		exit 1; \
	}

status: ## Show status of all containers and images
	@echo "$(BLUE)R2D3 Container Status:$(NC)"
	@echo ""
	@docker ps --format "table {{.Names}}\t{{.Status}}\t{{.Ports}}" --filter "name=r2d3-" || \
		echo "$(YELLOW)No R2D3 containers running.$(NC)"
	@echo ""
	@echo "$(BLUE)R2D3 Images:$(NC)"
	@docker images --format "table {{.Repository}}\t{{.Tag}}\t{{.Size}}" --filter "reference=ros2-*-r2d3" || \
		echo "$(YELLOW)No R2D3 images found.$(NC)"

clean: ## Clean up containers, images, and volumes
	@echo "$(RED)This will remove all R2D3 containers, images, and volumes!$(NC)"
	@read -p "Are you sure? (y/N): " confirm && [ "$$confirm" = "y" ] || exit 1
	@echo "$(BLUE)Cleaning up R2D3 environment...$(NC)"
	@if command -v docker-compose >/dev/null 2>&1; then \
		COMPOSE_CMD="docker-compose"; \
	else \
		COMPOSE_CMD="docker compose"; \
	fi; \
	$$COMPOSE_CMD --profile foxy --profile humble --profile jazzy down -v 2>/dev/null || true
	@docker images --format "{{.Repository}}:{{.Tag}}" --filter "reference=ros2-*-r2d3" | xargs -r docker rmi -f
	@docker volume ls --format "{{.Name}}" --filter "name=r2d3_" | xargs -r docker volume rm
	@echo "$(GREEN)R2D3 environment cleaned up!$(NC)"

fix-permissions: ## Fix permission issues in existing containers
	@echo "$(BLUE)Fixing permissions in R2D3 containers...$(NC)"
	@if docker ps --format "table {{.Names}}" | grep -q "^r2d3-$(DISTRO)$$"; then \
		echo "$(BLUE)Container r2d3-$(DISTRO) is running, fixing permissions...$(NC)"; \
		docker exec r2d3-$(DISTRO) sudo chown -R ros:ros /home/ros 2>/dev/null || true; \
		docker exec r2d3-$(DISTRO) sudo chmod -R 755 /home/ros 2>/dev/null || true; \
		docker exec r2d3-$(DISTRO) sudo mkdir -p /home/ros/ros2_ws/src; \
		docker exec r2d3-$(DISTRO) sudo chown -R ros:ros /home/ros/ros2_ws; \
		docker exec r2d3-$(DISTRO) sudo chmod -R 755 /home/ros/ros2_ws; \
		echo "$(GREEN)Permissions fixed! ros user now has full access to all files.$(NC)"; \
	else \
		echo "$(YELLOW)Container r2d3-$(DISTRO) is not running. Start it first with: make start DISTRO=$(DISTRO)$(NC)"; \
	fi

# Distribution-specific shortcuts
start-foxy: ## Start Foxy environment
	@$(MAKE) start DISTRO=foxy

start-humble: ## Start Humble environment  
	@$(MAKE) start DISTRO=humble

start-jazzy: ## Start Jazzy environment
	@$(MAKE) start DISTRO=jazzy

build-all: ## Build all distribution images
	@echo "$(BLUE)Building all R2D3 images...$(NC)"
	@if command -v docker-compose >/dev/null 2>&1; then \
		COMPOSE_CMD="docker-compose"; \
	else \
		COMPOSE_CMD="docker compose"; \
	fi; \
	HOSTNAME=$(HOSTNAME) $$COMPOSE_CMD build --no-cache
	@echo "$(GREEN)All R2D3 images built!$(NC)"

# Development shortcuts
dev: start shell ## Start environment and open shell (development mode)

demo: ## Start demo after ensuring environment is running
	@if ! docker ps --format "table {{.Names}}" | grep -q "^r2d3-$(DISTRO)$$"; then \
		echo "$(BLUE)Starting R2D3 $(DISTRO) environment first...$(NC)"; \
		$(MAKE) start DISTRO=$(DISTRO); \
		sleep 5; \
	fi
	@echo "$(BLUE)Launching R2D3 demo in $(DISTRO) container...$(NC)"
	@docker exec -it r2d3-$(DISTRO) bash -c "source /opt/ros/$(DISTRO)/setup.bash && source ~/ros2_ws/install/setup.bash && ros2 launch ros2_total_demo total_demo.launch.py"

camera: ## Start camera after ensuring environment is running
	@if ! docker ps --format "table {{.Names}}" | grep -q "^r2d3-$(DISTRO)$$"; then \
		echo "$(BLUE)Starting R2D3 $(DISTRO) environment first...$(NC)"; \
		$(MAKE) start DISTRO=$(DISTRO); \
		sleep 5; \
	fi
	@echo "$(BLUE)Launching camera in $(DISTRO) container...$(NC)"
	@docker exec -it r2d3-$(DISTRO) bash -c "source /opt/ros/$(DISTRO)/setup.bash && source ~/ros2_ws/install/setup.bash && ros2 launch realsense2_camera rs_launch.py"
