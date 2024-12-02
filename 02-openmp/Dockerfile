# Use uma imagem base com um compilador C++ e suporte a OpenMP
FROM ubuntu:22.04

# Instalar dependências
RUN apt-get update && apt-get install -y \
    build-essential \
    libomp-dev \
    && rm -rf /var/lib/apt/lists/*

# Criar diretório de trabalho
WORKDIR /app

# Copiar os arquivos de código-fonte
COPY . /app

# Compilar o código com OpenMP
RUN g++ -fopenmp -o openmp_app openmp.cpp -I/usr/include/nlohmann -std=c++11

# Comando padrão para executar o aplicativo
CMD ["./openmp_app", "input_graph.txt"]
