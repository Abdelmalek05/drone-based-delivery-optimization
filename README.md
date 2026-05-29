# Algiers Drone Delivery Optimization

This project builds a small Algiers-focused dataset and utility layer for drone delivery routing experiments. It combines real neighborhood coordinates, synthetic customer and fleet generation, Haversine distance calculations, and OpenStreetMap data fetching for buildings and roads.

## What is included

- `algiers_dataset_workflow.ipynb` - notebook workflow for exploring and assembling the dataset
- `src/algiers_data_generator.py` - generates customers, drone fleet data, and the full Algiers delivery dataset
- `src/distance_calculator.py` - computes Haversine distances and distance matrices
- `src/osm_data_fetcher.py` - fetches OpenStreetMap buildings and roads for Algiers and converts buildings into obstacle records
- `requirements.txt` - Python dependencies used by the project
- `data/` - stored dataset artifacts and backups

## Project data model

The generator focuses on:

- Real Algiers neighborhood coordinates
- A central depot at the Port of Algiers
- Drone constraints such as payload, battery, distance, and speed
- No-fly zones and land-boundary checks
- Synthetic customers with delivery priorities and package weights

## Dependencies

The project targets Python 3.8+ and uses these main packages:

- `osmnx`
- `geopandas`
- `shapely`
- `folium`
- `requests`

Install them with:

```bash
pip install -r requirements.txt
```

## How to use

### 1. Generate the Algiers dataset

Run the dataset generator directly:

```bash
python src/algiers_data_generator.py
```

By default this creates `algiers_delivery_dataset.json` with 20 customers and 5 drones.

### 2. Fetch OpenStreetMap data

To download buildings and roads for Algiers and save them under `data/`:

```bash
python src/osm_data_fetcher.py
```

### 3. Use the distance utilities

Import the helper functions in your own scripts or notebook:

```python
from src.distance_calculator import haversine_distance, calculate_distance_matrix
```

## Data files

The `data/` folder currently contains backup dataset files:

- `algiers_dataset_with_osm.backup.json`
- `algiers_dataset_with_osm.backup.json.orig`

## Notebook workflow

The notebook appears to be the main working area for assembling and inspecting the dataset. It has cells for loading dependencies, working with the generator, and reviewing the produced outputs, but it has not been executed yet.

## Notes

- The OSM fetcher uses live OpenStreetMap queries, so network access is required.
- Generated files may be large depending on the number of buildings and roads fetched.
- The project is organized as a lightweight research/workflow repo rather than a packaged application.
