# Use uma imagem base com um compilador C++ e suporte a PThreads
FROM ubuntu:22.04

# Instalar dependências
RUN apt-get update && apt-get install -y \
    build-essential \
    libpthread-stubs0-dev \
    && rm -rf /var/lib/apt/lists/*

# Criar diretório de trabalho
WORKDIR /app

# Copiar os arquivos de código-fonte
COPY . /app

# Compilar o código com PThreads
RUN g++ -o pthreads_app pthreads.cpp -lpthread -I/usr/include/nlohmann -std=c++11

# Comando padrão para executar o aplicativo
CMD ["./pthreads_app", "input_graph.txt"]
