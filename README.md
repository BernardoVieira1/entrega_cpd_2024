# Entrega CPD 2024

## Estrutura do Projeto
entrega_cpd_2024/\
    ├── 01-pthreads/\
    ├── 02-openmp/\
    ├── 03-openmpi/\
    └── README.md

## Como Utilizar

### Pré-requisitos

- **Docker** instalado na sua máquina. [Instalar Docker](https://docs.docker.com/get-docker/)
- **Docker Compose** (opcional, se necessário para orquestração).

### 01-pthreads

Implementação paralela utilizando **PThreads**.

#### Construir a Imagem Docker

```bash
cd 01-pthreads
docker build -t pthreads_app .

docker run --rm -v $(pwd)/input_graph.txt:/app/input_graph.txt pthreads_app

cd 02-openmp
docker build -t openmp_app .

docker run --rm -v $(pwd)/input_graph.txt:/app/input_graph.txt openmp_app


cd 03-openmpi
docker build -t openmpi_app .

docker run --rm -v $(pwd)/input_graph.txt:/app/input_graph.txt openmpi_app
