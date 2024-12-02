#include <iostream>
#include <fstream>
#include <sstream>
#include <vector>
#include <unordered_set>
#include <unordered_map>
#include <queue>
#include <stack>
#include <cmath>
#include <omp.h>
#include "json.hpp"

using json = nlohmann::json;

// Estruturas globais
std::unordered_set<int> nodes;
std::unordered_map<int, std::vector<int>> adjList;
std::unordered_map<int, std::vector<int>> reverseAdjList;
size_t edges = 0;
std::vector<std::pair<int, int>> graphData;

// Lista de adjacência não direcionada
std::unordered_map<int, std::unordered_set<int>> undirectedAdjList;

// Função para carregar o grafo
void loadGraph(size_t start, size_t end) {
    std::unordered_set<int> localNodes;
    size_t localEdges = 0;

    for (size_t i = start; i < end; ++i) {
        int from = graphData[i].first;
        int to = graphData[i].second;

        localNodes.insert(from);
        localNodes.insert(to);
        localEdges++;

        #pragma omp critical
        {
            adjList[from].push_back(to);
            reverseAdjList[to].push_back(from);
            undirectedAdjList[from].insert(to);
            undirectedAdjList[to].insert(from);
        }
    }

    #pragma omp critical
    {
        nodes.insert(localNodes.begin(), localNodes.end());
        edges += localEdges;
    }
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

            if (componentNodes > largestSCCNodes) {
                largestSCCNodes = componentNodes;
                largestSCCEdges = componentEdges;
                largestSCCSet = componentSet;
            }
        }
    }

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

// Função para calcular as métricas
json calculateMetrics() {
    auto [largestWCCNodes, largestWCCEdges] = calculateWCC();
    auto [largestSCCNodes, largestSCCEdges] = calculateSCC();

    return json{
        {"graph_metrics", {
            {"nodes", nodes.size()},
            {"edges", edges},
            {"largest_wcc", {
                {"nodes", largestWCCNodes},
                {"edges", largestWCCEdges}
            }},
            {"largest_scc", {
                {"nodes", largestSCCNodes},
                {"edges", largestSCCEdges}
            }}
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

    size_t numThreads = 8;
    size_t chunkSize = graphData.size() / numThreads;

    #pragma omp parallel for
    for (size_t i = 0; i < numThreads; ++i) {
        size_t start = i * chunkSize;
        size_t end = (i == numThreads - 1) ? graphData.size() : start + chunkSize;
        loadGraph(start, end);
    }

    json result = calculateMetrics();
    std::ofstream outfile("graph_metrics.json");
    outfile << result.dump(4);
    outfile.close();

    std::cout << "Métricas do grafo salvas em graph_metrics.json" << std::endl;

    return 0;
}
