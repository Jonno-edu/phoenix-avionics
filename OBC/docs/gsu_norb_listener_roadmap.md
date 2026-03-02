# GSU Web UI: nORB Listener Terminal Roadmap

## Overview
This document outlines the roadmap for implementing a live nORB message listener within the Ground Station Unit (GSU) Web UI. This feature will allow operators to select specific nORB topics from a side panel and view their live data streams in dedicated, nicely formatted terminal windows. This is crucial for real-time debugging, telemetry monitoring, and system verification.

## Key Features
- **Topic Discovery:** A side panel listing all available/active nORB topics being telemetered from the OBC.
- **Multi-Terminal Interface:** Support for opening multiple terminal windows or split panes to monitor different nORB topics simultaneously.
- **Formatted Output:** Data streams will be parsed and displayed in a highly readable format (e.g., color-coded JSON, tabular, or key-value pairs) rather than raw byte streams.
- **Stream Controls:** Ability to pause, resume, clear, and filter the incoming message streams for easier debugging and monitoring.

## Architecture & Data Flow
1. **OBC (Flight Software):** Publishes nORB messages. A telemetry/datalink module subscribes to requested topics and transmits them over the radio link to the GSU.
2. **GSU Backend:** Receives the telemetry stream, decodes the nORB messages, and exposes them via a WebSocket connection to the frontend.
3. **GSU Frontend (Web UI):** Connects to the WebSocket, provides the UI for topic selection, and routes incoming message data to the appropriate terminal UI components.

## Implementation Phases

### Phase 1: Backend Infrastructure & Data Routing
- [ ] Implement a WebSocket server on the GSU backend to stream telemetry data to the web client.
- [ ] Create a mechanism to decode incoming nORB messages from the datalink into structured JSON objects.
- [ ] Establish a protocol for the frontend to request specific topics (subscribe/unsubscribe) to save bandwidth over the WebSocket and radio link.

### Phase 2: Basic Frontend Terminal
- [ ] Integrate a web-based terminal emulator (e.g., `xterm.js`) or a custom auto-scrolling log component into the GSU Web UI.
- [ ] Establish the WebSocket connection in the frontend and pipe a single, hardcoded nORB topic into the terminal.
- [ ] Implement basic formatting (pretty-printing JSON or key-value pairs with syntax highlighting).

### Phase 3: UI Integration & Topic Selection
- [ ] Build the side panel component to display available nORB topics dynamically.
- [ ] Implement click-to-listen functionality: clicking a topic in the side panel opens a terminal and sends a subscribe request to the backend.
- [ ] Add basic stream controls to the terminal UI (Pause, Resume, Clear).

### Phase 4: Multi-Window & Advanced Features
- [ ] Upgrade the UI to support a window manager or split-pane layout (e.g., using `GoldenLayout`, `react-grid-layout`, or a custom flexbox implementation).
- [ ] Allow users to drag and drop topics from the side panel into different panes to spawn new listener terminals.
- [ ] Add advanced formatting options (e.g., highlighting specific thresholds, filtering out unchanged fields to reduce noise).
- [ ] Implement search and regex filtering within the terminal windows to isolate specific data points.
