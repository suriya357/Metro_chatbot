# Metro_chatbot

# Chennai Metro Route Finder

This is a responsive web application designed to help users find the optimal route between two Chennai Metro stations using either Breadth-First Search (BFS) or Depth-First Search (DFS) algorithms. The application provides a clean, modern user interface with both a map view and an interactive graph visualization of the metro network.

## 🌟 Features

* **Station Selection:** Easily select "From" and "To" metro stations using intuitive dropdown menus.
* **Algorithm Choice:** Choose between BFS and DFS algorithms to find the route.
* **Route Details:** Displays the sequence of stations in the path, the total number of stations, estimated total distance, and estimated travel time.
* **Interactive Map View:** Visualizes the metro network and the calculated route on an interactive Leaflet map, highlighting start, end, and intermediate stations.
* **Dynamic Graph View:** Provides a canvas-based graph representation of the metro network.
    * **Path Highlighting:** The calculated path is highlighted in red.
    * **Start/End Markers:** Start node is marked in green, and the end node in dark red.
    * **Animated Traversal:** Observe the step-by-step traversal of the chosen algorithm (BFS/DFS) on the graph.
    * **Panning:** Drag and move the graph view to explore different parts of the network.
* **Responsive Design:** The layout adapts seamlessly to various screen sizes, offering a two-column layout on larger screens and a stacked layout on smaller devices.
* **Modern UI:** Clean, modern interface with rounded buttons and a consistent blue color scheme.
* **Chat Box Integration:** A chat-like interface provides updates and route information.

## 🛠️ Technologies Used

### Backend (Flask - Python)

* **Flask:** A micro web framework for Python, used to serve the web application and handle route calculation requests.
* **NetworkX:** A Python library for the creation, manipulation, and study of the structure, dynamics, and functions of complex networks. Used for building the metro graph and running BFS/DFS algorithms.

### Frontend (HTML, CSS, JavaScript)

* **HTML5:** Structure of the web application.
* **CSS3:** Styling and responsive layout (`style.css`).
* **JavaScript (ES6+):**
    * **Leaflet.js:** An open-source JavaScript library for mobile-friendly interactive maps, used for the map view.
    * **Custom Canvas Drawing:** JavaScript is used to dynamically draw and animate the metro graph on an HTML `<canvas>` element.
    * **Fetch API:** For asynchronous communication with the Flask backend.
    * **DOM Manipulation:** For dynamic updates to the UI.

## 🚀 Setup and Installation

To get this application up and running on your local machine, follow these steps:

### 1. Prerequisites

Make sure you have the following installed:
* **Python 3.x**
* **pip** (Python package installer, usually comes with Python)

### 2. Clone the Repository

(Assuming this code is part of a repository)

```bash
git clone <your-repository-url>
cd chennai-metro-route-finder
