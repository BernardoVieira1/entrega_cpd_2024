# Use uma imagem base com OpenMPI instalado
FROM ubuntu:22.04

# Instalar dependências
RUN apt-get update && apt-get install -y \
    build-essential \
    openmpi-bin \
    libopenmpi-dev \
    && rm -rf /var/lib/apt/lists/*

# Criar diretório de trabalho
WORKDIR /app

# Copiar os arquivos de código-fonte
COPY . /app

# Compilar o código com OpenMPI
RUN mpic++ -o openmpi_app openmpi.cpp -std=c++11

# Comando padrão para executar o aplicativo
CMD ["mpirun", "-np", "4", "./openmpi_app", "input_graph.txt"]
