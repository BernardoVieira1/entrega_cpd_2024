#include <iostream>
#include <fstream>
#include <sstream>
#include <vector>
#include <unordered_set>
#include <unordered_map>
#include <queue>
#include <stack>
#include <cmath>
#include <pthread.h>
#include <mutex>
#include <nlohmann/json.hpp>

using json = nlohmann::json;

std::mutex mtx;
std::unordered_set<int> nodes;
std::unordered_map<int, std::vector<int>> adjList;
std::unordered_map<int, std::vector<int>> reverseAdjList;
size_t edges = 0;
std::vector<std::pair<int, int>> graphData;

// Adicionar esta variável global para a lista de adjacência não direcionada
std::unordered_map<int, std::unordered_set<int>> undirectedAdjList;

// Função para carregar o grafo
void* loadGraph(void* arg) {
    size_t start = ((size_t*)arg)[0];
    size_t end = ((size_t*)arg)[1];

    std::unordered_set<int> localNodes;
    size_t localEdges = 0;

    for (size_t i = start; i < end; ++i) {
        int from = graphData[i].first;
        int to = graphData[i].second;

        localNodes.insert(from);
        localNodes.insert(to);
        localEdges++;

        mtx.lock();
        adjList[from].push_back(to);
        reverseAdjList[to].push_back(from);
        // Construir a lista de adjacência não direcionada
        undirectedAdjList[from].insert(to);
        undirectedAdjList[to].insert(from);
        mtx.unlock();
    }

    mtx.lock();
    nodes.insert(localNodes.begin(), localNodes.end());
    edges += localEdges;
    mtx.unlock();

    pthread_exit(nullptr);
}

// Função para calcular o maior componente fracamente conectado (WCC)
std::pair<size_t, size_t> calculateWCC() {
    std::unordered_set<int> visited;
    size_t largestWCCNodes = 0, largestWCCEdges = 0;

    for (int node : nodes) {
        if (visited.find(node) != visited.end()) continue;

        std::queue<int> q;
        q.push(node);
        visited.insert(node);

        size_t componentNodes = 0, componentEdges = 0;
        while (!q.empty()) {
            int curr = q.front();
            q.pop();
            componentNodes++;

            for (int neighbor : adjList[curr]) {
                componentEdges++;
                if (visited.find(neighbor) == visited.end()) {
                    visited.insert(neighbor);
                    q.push(neighbor);
                }
            }
            for (int neighbor : reverseAdjList[curr]) {
                componentEdges++;
                if (visited.find(neighbor) == visited.end()) {
                    visited.insert(neighbor);
                    q.push(neighbor);
                }
            }
        }

        if (componentNodes > largestWCCNodes) {
            largestWCCNodes = componentNodes;
            largestWCCEdges = componentEdges / 2;
        }
    }

    return {largestWCCNodes, largestWCCEdges};
}

// Função para calcular o maior componente fortemente conectado (SCC) usando Kosaraju
std::pair<size_t, size_t> calculateSCC() {
    std::unordered_set<int> visited;
    std::stack<int> finishStack;

    // 1ª Passagem: Ordenar nós por tempos de término (DFS no grafo original)
    for (int node : nodes) {
        if (visited.find(node) == visited.end()) {
            std::stack<int> dfsStack;
            dfsStack.push(node);

            while (!dfsStack.empty()) {
                int curr = dfsStack.top();
                dfsStack.pop();

                if (visited.find(curr) != visited.end()) continue;
                visited.insert(curr);

                for (int neighbor : adjList[curr]) {
                    if (visited.find(neighbor) == visited.end()) {
                        dfsStack.push(neighbor);
                    }
                }

                finishStack.push(curr);
            }
        }
    }

    // 2ª Passagem: Processar o grafo reverso
    visited.clear();
    size_t largestSCCNodes = 0, largestSCCEdges = 0;
    std::unordered_set<int> largestSCCSet;

    while (!finishStack.empty()) {
        int node = finishStack.top();
        finishStack.pop();

        if (visited.find(node) == visited.end()) {
            std::stack<int> dfsStack;
            dfsStack.push(node);

            size_t componentNodes = 0;
            std::unordered_set<int> componentSet;
            size_t componentEdges = 0;

            while (!dfsStack.empty()) {
                int curr = dfsStack.top();
                dfsStack.pop();

                if (visited.find(curr) != visited.end()) continue;
                visited.insert(curr);
                componentSet.insert(curr);
                componentNodes++;

                for (int neighbor : reverseAdjList[curr]) {
                    if (componentSet.find(neighbor) != componentSet.end()) {
                        componentEdges++;
                    }
                    if (visited.find(neighbor) == visited.end()) {
                        dfsStack.push(neighbor);
                    }
                }
            }

            // Atualizar o maior SCC
            if (componentNodes > largestSCCNodes) {
                largestSCCNodes = componentNodes;
                largestSCCEdges = componentEdges;
                largestSCCSet = componentSet;
            }
        }
    }

    // Recontar as arestas internas ao SCC para evitar contagem duplicada
    size_t actualEdgesInSCC = 0;
    for (int node : largestSCCSet) {
        for (int neighbor : adjList[node]) {
            if (largestSCCSet.find(neighbor) != largestSCCSet.end()) {
                actualEdgesInSCC++;
            }
        }
    }

    largestSCCEdges = actualEdgesInSCC;

    return {largestSCCNodes, largestSCCEdges};
}

// Função para calcular o número de triângulos em um grafo não direcionado
std::pair<size_t, double> calculateTriangles() {
    size_t triangles = 0;

    for (const auto& [u, neighbors_u] : undirectedAdjList) {
        for (int v : neighbors_u) {
            if (v <= u) continue; // Garantir que cada par seja considerado apenas uma vez
            const std::unordered_set<int>& neighbors_v = undirectedAdjList[v];
            for (int w : neighbors_u) {
                if (w <= v) continue; // Garantir w > v > u para evitar duplicatas
                if (neighbors_v.find(w) != neighbors_v.end()) {
                    // Triângulo encontrado entre u, v, w
                    triangles++;
                }
            }
        }
    }

    // Calcular o número de tripletos conectados (abertos e fechados)
    size_t triplets = 0;
    for (const auto& [node, neighbors] : undirectedAdjList) {
        size_t degree = neighbors.size();
        if (degree >= 2) {
            triplets += degree * (degree - 1) / 2;
        }
    }

    double closedTriangleFraction = (triplets > 0) ? (double)(3 * triangles) / triplets : 0.0;

    return {triangles, closedTriangleFraction};
}

// Função para calcular o coeficiente de agrupamento médio
double calculateAverageClusteringCoefficient() {
    double totalClusteringCoefficient = 0.0;
    size_t nodeCount = 0;

    for (const auto& [node, neighbors] : undirectedAdjList) {
        size_t degree = neighbors.size();
        if (degree < 2) continue;
        nodeCount++;

        size_t connectedNeighborPairs = 0;
        for (int neighbor1 : neighbors) {
            for (int neighbor2 : neighbors) {
                if (neighbor1 < neighbor2 && undirectedAdjList[neighbor1].find(neighbor2) != undirectedAdjList[neighbor1].end()) {
                    connectedNeighborPairs++;
                }
            }
        }
        double clusteringCoefficient = (2.0 * connectedNeighborPairs) / (degree * (degree - 1));
        totalClusteringCoefficient += clusteringCoefficient;
    }

    return (nodeCount > 0) ? totalClusteringCoefficient / nodeCount : 0.0;
}

// Função para calcular o diâmetro e o diâmetro efetivo
std::pair<int, double> calculateDiameters() {
    int diameter = 0;
    std::vector<int> shortestPaths;

    // Realizar BFS a partir de um subconjunto de nós (para desempenho)
    int numSamples = 100;
    std::vector<int> sampleNodes;
    for (int node : nodes) {
        sampleNodes.push_back(node);
        if (sampleNodes.size() >= numSamples) break;
    }

    for (int startNode : sampleNodes) {
        std::unordered_map<int, int> distances;
        std::queue<int> q;
        distances[startNode] = 0;
        q.push(startNode);

        while (!q.empty()) {
            int curr = q.front();
            q.pop();

            for (int neighbor : undirectedAdjList[curr]) {
                if (distances.find(neighbor) == distances.end()) {
                    distances[neighbor] = distances[curr] + 1;
                    q.push(neighbor);
                    diameter = std::max(diameter, distances[neighbor]);
                    shortestPaths.push_back(distances[neighbor]);
                }
            }
        }
    }

    // Calcular o diâmetro efetivo (90º percentil)
    double effectiveDiameter = 0.0;
    if (!shortestPaths.empty()) {
        std::sort(shortestPaths.begin(), shortestPaths.end());
        size_t idx = static_cast<size_t>(0.9 * shortestPaths.size());
        effectiveDiameter = shortestPaths[idx];
    }

    return {diameter, effectiveDiameter};
}

// Função para calcular as métricas
json calculateMetrics() {
    auto [largestWCCNodes, largestWCCEdges] = calculateWCC();
    auto [largestSCCNodes, largestSCCEdges] = calculateSCC();
    auto [triangleCount, closedTriangleFraction] = calculateTriangles();
    double averageClusteringCoefficient = calculateAverageClusteringCoefficient();
    auto [diameter, effectiveDiameter] = calculateDiameters();

    return json{
        {"graph_metrics", {
            {"nodes", nodes.size()},
            {"edges", edges},
            {"largest_wcc", {
                {"nodes", largestWCCNodes},
                {"fraction_of_total_nodes", (double)largestWCCNodes / nodes.size()},
                {"edges", largestWCCEdges},
                {"fraction_of_total_edges", (double)largestWCCEdges / edges}
            }},
            {"largest_scc", {
                {"nodes", largestSCCNodes},
                {"fraction_of_total_nodes", (double)largestSCCNodes / nodes.size()},
                {"edges", largestSCCEdges},
                {"fraction_of_total_edges", (double)largestSCCEdges / edges}
            }},
            {"average_clustering_coefficient", averageClusteringCoefficient},
            {"triangles", triangleCount},
            {"fraction_of_closed_triangles", closedTriangleFraction},
            {"diameter", diameter},
            {"effective_diameter_90_percentile", effectiveDiameter}
        }}
    };
}

int main(int argc, char* argv[]) {
    if (argc < 2) {
        std::cerr << "Uso: " << argv[0] << " <arquivo_de_entrada>" << std::endl;
        return 1;
    }

    std::ifstream infile(argv[1]);
    if (!infile.is_open()) {
        std::cerr << "Erro: Não foi possível abrir o arquivo " << argv[1] << std::endl;
        return 1;
    }

    std::string line;
    while (std::getline(infile, line)) {
        if (line[0] == '#') continue;
        std::istringstream iss(line);
        int from, to;
        if (iss >> from >> to) {
            graphData.emplace_back(from, to);
        }
    }
    infile.close();

    int numThreads = 8;
    pthread_t threads[numThreads];
    size_t chunkSize = graphData.size() / numThreads;

    for (int i = 0; i < numThreads; ++i) {
        size_t start = i * chunkSize;
        size_t end = (i == numThreads - 1) ? graphData.size() : start + chunkSize;

        size_t* args = new size_t[2]{start, end};
        if (pthread_create(&threads[i], nullptr, loadGraph, args) != 0) {
            std::cerr << "Erro ao criar a thread " << i << std::endl;
            return 1;
        }
    }

    for (int i = 0; i < numThreads; ++i) {
        pthread_join(threads[i], nullptr);
    }

    json result = calculateMetrics();
    std::ofstream outfile("/app/data/graph_metrics.json");
    outfile << result.dump(4);
    outfile.close();

    std::cout << "Métricas do grafo salvas em graph_metrics.json" << std::endl;

    return 0;
}
