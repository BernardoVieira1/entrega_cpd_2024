# Use uma imagem base do Ubuntu
FROM ubuntu:22.04

# Evitar interações durante a instalação de pacotes
ENV DEBIAN_FRONTEND=noninteractive

# Atualizar os repositórios e instalar dependências
RUN apt-get update && apt-get install -y \
    build-essential \
    openmpi-bin \
    libopenmpi-dev \
    nlohmann-json3-dev \
    sudo \
    && rm -rf /var/lib/apt/lists/*

# Criar diretórios para a aplicação e para os dados
RUN mkdir -p /app/bin /app/data

# Criar um usuário não-root chamado 'mpiuser'
RUN useradd -ms /bin/bash mpiuser \
    && echo "mpiuser ALL=(ALL) NOPASSWD:ALL" >> /etc/sudoers

# Definir o proprietário dos diretórios para o usuário não-root
RUN chown -R mpiuser:mpiuser /app/bin /app/data

# Definir o diretório de trabalho para a aplicação
WORKDIR /app/bin

# Copiar os arquivos de código-fonte para o diretório da aplicação
COPY openmpi.cpp /app/bin/

# Compilar o código com OpenMPI
RUN mpic++ -o openmpi_app openmpi.cpp -std=c++11

# Garantir permissões de execução
RUN chmod +x openmpi_app

# Alterar para o usuário não-root 'mpiuser'
USER mpiuser

# Definir o comando padrão para executar o aplicativo, apontando para o arquivo de entrada no diretório de dados
CMD ["mpirun", "-np", "4", "./openmpi_app", "/app/data/input_graph.txt"]
