Installation Guide
==================

This guide will help you install and set up ROBIN on your system.

Requirements
------------

System Requirements
~~~~~~~~~~~~~~~~~~~

* **Operating System**: Ubuntu 20.04+ / macOS 10.15+ / Windows 10+
* **Python**: 3.12 or higher
* **Memory**: Minimum 4GB RAM (8GB recommended)
* **Storage**: At least 5GB free space
* **Docker & Docker Compose**: For FIWARE services

Software Dependencies
~~~~~~~~~~~~~~~~~~~~~

* **Docker & Docker Compose** (for FIWARE stack)
* **Poetry** (for Python dependency management)
* **Git** for version control

Installation Methods
--------------------

Method 1: Using Poetry (Recommended)
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

.. code-block:: bash

   # Clone the repository
   git clone https://github.com/Industry40Lab/open-robin.git
   cd open-robin

   # Install Poetry if not already installed
   curl -sSL https://install.python-poetry.org | python3 -

   # Install Python dependencies
   poetry install

   # Activate the virtual environment
   poetry shell

Method 2: Using pip
~~~~~~~~~~~~~~~~~~~

.. code-block:: bash

   # Clone the repository
   git clone https://github.com/Industry40Lab/open-robin.git
   cd open-robin

   # Create a virtual environment
   python -m venv venv
   source venv/bin/activate  # On Windows: venv\Scripts\activate

   # Install dependencies
   pip install -e .

Method 3: Docker Installation
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

.. code-block:: bash

   # Pull the Docker image
   docker pull industry40lab/open-robin:latest

   # Run the container with FIWARE services
   docker compose up -d

   # Run the application container
   docker run -it --rm --network container:fiware-orion industry40lab/open-robin:latest

FIWARE Services Setup
---------------------

Docker Compose (Recommended)
~~~~~~~~~~~~~~~~~~~~~~~~~~~~

.. code-block:: bash

   # Start FIWARE services
   docker compose up -d

   # Verify services are running
   docker compose ps

This will start:
* **Orion-LD Context Broker** (port 1026)
* **MongoDB** (port 27017)
* **TimescaleDB** (host port 5433 → container 5432)
* **Mintaka** (host port 9090 → container 8080)

Manual Installation
~~~~~~~~~~~~~~~~~~~

.. code-block:: bash

   # Start Orion-LD Context Broker
   docker run -d --name orion-ld -p 1026:1026 fiware/orion-ld:1.8.0

   # Start MongoDB
   docker run -d --name mongo-db -p 27017:27017 mongo:4.4

   # Start TimescaleDB (example, minimal config)
   docker run -d --name timescaledb -e POSTGRES_DB=orion -e POSTGRES_USER=orion -e POSTGRES_PASSWORD=orionpass -p 5433:5432 timescale/timescaledb-ha:pg15-latest

   # Start Mintaka (reads Orion-LD temporal store in TimescaleDB)
   docker run -d --name mintaka -p 9090:8080 \
     -e DATASOURCES_DEFAULT_URL=jdbc:postgresql://host.docker.internal:5433/orion \
     -e DATASOURCES_DEFAULT_USERNAME=orion \
     -e DATASOURCES_DEFAULT_PASSWORD=orionpass \
     -e MINTAKA_BROKER_URL=http://orion-ld:1026 \
     -e MICRONAUT_MULTITENANCY_TENANTRESOLVER_HTTPHEADER_ENABLED=true \
     -e MICRONAUT_MULTITENANCY_TENANTRESOLVER_HTTPHEADER_NAMES_0=NGSILD-Tenant \
     fiware/mintaka:latest

Verification
------------

Test Installation
~~~~~~~~~~~~~~~~~

.. code-block:: bash

   # Check system status
   poetry run robin status

   # Test FIWARE connections
   curl -X GET 'http://localhost:1026/version'

   # Test Mintaka temporal API
   curl -X GET 'http://localhost:9090/temporal/entities?limit=1'

Environment Setup
~~~~~~~~~~~~~~~~~

The FIWARE services will be accessible at:

* **Orion-LD**: http://localhost:1026
* **Mintaka**: http://localhost:9090
* **TimescaleDB**: host port 5433 (if exposed)

Troubleshooting
---------------

If you hit issues during installation, see the :doc:`reference/troubleshooting`
guide for common problems and solutions.

Next Steps
----------

After installation, proceed to the :doc:`quickstart` guide to begin using ROBIN. 
