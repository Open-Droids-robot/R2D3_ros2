#!/bin/bash

# R2D3 Robot Management Script
# Convenient wrapper for Docker Compose operations

set -e

# Colors for output
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
BLUE='\033[0;34m'
NC='\033[0m' # No Color

# Script directory
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PROJECT_DIR="$(dirname "$SCRIPT_DIR")"

# Change to project directory
cd "$PROJECT_DIR"

# Helper functions
log_info() {
    echo -e "${BLUE}[INFO]${NC} $1"
}

log_success() {
    echo -e "${GREEN}[SUCCESS]${NC} $1"
}

log_warning() {
    echo -e "${YELLOW}[WARNING]${NC} $1"
}

log_error() {
    echo -e "${RED}[ERROR]${NC} $1"
}

show_help() {
    cat << EOF
R2D3 Robot Management Script

Usage: $0 [COMMAND] [DISTRO] [OPTIONS]

Commands:
  start [distro]     Start R2D3 robot environment
  stop [distro]      Stop R2D3 robot environment
  restart [distro]   Restart R2D3 robot environment
  build [distro]     Build R2D3 robot images
  shell [distro]     Open shell in running container
  logs [distro]      Show container logs
  clean              Clean up containers and volumes
  status             Show status of all containers
  help               Show this help message

Distributions:
  foxy               ROS2 Foxy (Original R2D3)
  humble             ROS2 Humble (Recommended, Default)
  jazzy              ROS2 Jazzy (Latest)

Examples:
  $0 start           # Start default (Humble) environment
  $0 start foxy      # Start Foxy environment
  $0 build humble    # Build Humble image
  $0 shell           # Open shell in default container
  $0 clean           # Clean up everything

Environment Setup:
  Make sure to run these commands on the host before starting:
  xhost +local:docker
  pulseaudio --start

EOF
}

# Check prerequisites
check_prerequisites() {
    # Check if Docker is running
    if ! docker info >/dev/null 2>&1; then
        log_error "Docker is not running. Please start Docker first."
        exit 1
    fi

    # Check if Docker Compose is available
    if ! command -v docker-compose >/dev/null 2>&1 && ! docker compose version >/dev/null 2>&1; then
        log_error "Docker Compose is not available. Please install Docker Compose."
        exit 1
    fi

    # Check if .env file exists, create from example if not
    if [ ! -f .env ]; then
        if [ -f env.example ]; then
            log_info "Creating .env file from env.example"
            cp env.example .env
        else
            log_warning ".env file not found. Using default values."
        fi
    fi

    # Check X11 forwarding
    if [ -z "$DISPLAY" ]; then
        log_warning "DISPLAY environment variable not set. GUI applications may not work."
    fi
}

# Get Docker Compose command
get_compose_cmd() {
    if command -v docker-compose >/dev/null 2>&1; then
        echo "docker-compose"
    else
        echo "docker compose"
    fi
}

# Normalize distro name
normalize_distro() {
    local distro="$1"
    case "$distro" in
        ""|"default"|"humble")
            echo "humble"
            ;;
        "foxy")
            echo "foxy"
            ;;
        "jazzy")
            echo "jazzy"
            ;;
        *)
            log_error "Unknown distribution: $distro"
            log_info "Available distributions: foxy, humble, jazzy"
            exit 1
            ;;
    esac
}

# Setup host environment
setup_host() {
    log_info "Setting up host environment..."
    
    # Create PulseAudio directory
    mkdir -p "/run/user/$(id -u)/pulse" 2>/dev/null || true
    
    # Check X11 forwarding
    if command -v xhost >/dev/null 2>&1; then
        xhost +local:docker >/dev/null 2>&1 || log_warning "Failed to setup X11 forwarding"
    else
        log_warning "xhost not found. Install x11-xserver-utils for GUI support."
    fi
    
    # Check PulseAudio
    if command -v pulseaudio >/dev/null 2>&1; then
        if ! pgrep -x pulseaudio >/dev/null; then
            log_info "Starting PulseAudio..."
            pulseaudio --start 2>/dev/null || log_warning "Failed to start PulseAudio"
        fi
    else
        log_warning "PulseAudio not found. Audio may not work in containers."
    fi
}

# Main command handling
case "${1:-help}" in
    "start")
        check_prerequisites
        setup_host
        
        DISTRO=$(normalize_distro "${2:-humble}")
        COMPOSE_CMD=$(get_compose_cmd)
        
        log_info "Starting R2D3 $DISTRO environment..."
        
        if [ "$DISTRO" = "humble" ]; then
            $COMPOSE_CMD --profile default up -d
        else
            $COMPOSE_CMD --profile "$DISTRO" up -d
        fi
        
        log_success "R2D3 $DISTRO environment started!"
        log_info "Connect with: $0 shell $DISTRO"
        ;;
        
    "stop")
        DISTRO=$(normalize_distro "${2:-humble}")
        COMPOSE_CMD=$(get_compose_cmd)
        
        log_info "Stopping R2D3 $DISTRO environment..."
        
        if [ "$DISTRO" = "humble" ]; then
            $COMPOSE_CMD --profile default down
        else
            $COMPOSE_CMD --profile "$DISTRO" down
        fi
        
        log_success "R2D3 $DISTRO environment stopped!"
        ;;
        
    "restart")
        DISTRO=$(normalize_distro "${2:-humble}")
        
        log_info "Restarting R2D3 $DISTRO environment..."
        $0 stop "$DISTRO"
        sleep 2
        $0 start "$DISTRO"
        ;;
        
    "build")
        check_prerequisites
        
        DISTRO=$(normalize_distro "${2:-humble}")
        COMPOSE_CMD=$(get_compose_cmd)
        
        log_info "Building R2D3 $DISTRO image..."
        
        if [ "$DISTRO" = "humble" ]; then
            $COMPOSE_CMD --profile default build --no-cache
        else
            $COMPOSE_CMD --profile "$DISTRO" build --no-cache
        fi
        
        log_success "R2D3 $DISTRO image built!"
        ;;
        
    "shell")
        DISTRO=$(normalize_distro "${2:-humble}")
        CONTAINER_NAME="r2d3-$DISTRO"
        
        if ! docker ps --format "table {{.Names}}" | grep -q "^$CONTAINER_NAME$"; then
            log_error "Container $CONTAINER_NAME is not running."
            log_info "Start it with: $0 start $DISTRO"
            exit 1
        fi
        
        log_info "Opening shell in R2D3 $DISTRO container..."
        docker exec -it "$CONTAINER_NAME" bash
        ;;
        
    "logs")
        DISTRO=$(normalize_distro "${2:-humble}")
        CONTAINER_NAME="r2d3-$DISTRO"
        
        log_info "Showing logs for R2D3 $DISTRO container..."
        docker logs -f "$CONTAINER_NAME" 2>/dev/null || {
            log_error "Container $CONTAINER_NAME not found."
            exit 1
        }
        ;;
        
    "status")
        log_info "R2D3 Container Status:"
        echo
        docker ps --format "table {{.Names}}\t{{.Status}}\t{{.Ports}}" --filter "name=r2d3-" || {
            log_info "No R2D3 containers running."
        }
        echo
        
        log_info "R2D3 Images:"
        docker images --format "table {{.Repository}}\t{{.Tag}}\t{{.Size}}" --filter "reference=ros2-*-r2d3" || {
            log_info "No R2D3 images found."
        }
        ;;
        
    "clean")
        COMPOSE_CMD=$(get_compose_cmd)
        
        log_warning "This will remove all R2D3 containers, images, and volumes!"
        read -p "Are you sure? (y/N): " -n 1 -r
        echo
        
        if [[ $REPLY =~ ^[Yy]$ ]]; then
            log_info "Cleaning up R2D3 environment..."
            
            # Stop all profiles
            $COMPOSE_CMD --profile foxy --profile humble --profile jazzy down -v 2>/dev/null || true
            
            # Remove images
            docker images --format "{{.Repository}}:{{.Tag}}" --filter "reference=ros2-*-r2d3" | xargs -r docker rmi -f
            
            # Remove volumes
            docker volume ls --format "{{.Name}}" --filter "name=r2d3_" | xargs -r docker volume rm
            
            log_success "R2D3 environment cleaned up!"
        else
            log_info "Cleanup cancelled."
        fi
        ;;
        
    "help"|"-h"|"--help")
        show_help
        ;;
        
    *)
        log_error "Unknown command: $1"
        echo
        show_help
        exit 1
        ;;
esac
