# Quick start for local development

## Setup

1. Install [conda forge](https://conda-forge.org/download/)  and [pnpm](https://pnpm.io/installation) if you haven't already.

2. Create the conda environment and install dependencies:

```bash
mamba env create -f environment.yml
```

3. Install the frontend dependencies:

```bash
cd srt/svelte-frontend
pnpm install
```

4. Tell pnpm where to find the conda environment by creating a `.env` file in the `srt/svelte-frontend` directory with the following content:

```bash
SRT_DEV_PYTHON_PATH="/path/to/your/conda/env/bin/python"
```

You can copy the `.env.example` file and modify it accordingly. You can get the path to your conda environment by running:

```bash
conda run -n srt-dev python -c "import sys;print(sys.executable)"
```

## Running the application

1. Activate the environment:

```bash
mamba activate srt-dev
```

2. Run the development server:

```bash
python scripts/run_dev_server.py
```

3. Open your browser and go to [http://localhost:5173/monitor](http://localhost:5173/monitor) to see the application running.

4. Changes you make to the frontend code will be automatically reflected in the browser. Changes to the backend code will require restarting the development server (Ctrl+C to stop, then run the command again).
