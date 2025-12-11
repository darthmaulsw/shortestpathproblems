#include <iostream>
#include <fstream>
#include <sstream>
#include <vector>
#include <unordered_map>
#include <queue>
#include <cmath>
#include <string>
#include <limits>
#include <cctype>
#include <algorithm>
#include <iomanip>

using namespace std;

struct Coord {
    double lat;
    double lon;
};

struct CoordHash {
    size_t operator()(Coord const& c) const noexcept {
        auto h1 = std::hash<long long>{}(llround(c.lat * 1e7));
        auto h2 = std::hash<long long>{}(llround(c.lon * 1e7));
        return h1 ^ (h2 << 1);
    }
};

struct CoordEq {
    bool operator()(Coord const& a, Coord const& b) const noexcept {
        return fabs(a.lat - b.lat) < 1e-9 && fabs(a.lon - b.lon) < 1e-9;
    }
};

struct Edge {
    int to;
    double dist;
    string streetName;
    string roadType;
};

double heuristic(const Coord& a, const Coord& b) {
    double dlat = a.lat - b.lat;
    double dlon = a.lon - b.lon;
    return sqrt(dlat * dlat + dlon * dlon);
}

int findClosestNode(const vector<Coord>& nodes, const Coord& c) {
    if (nodes.empty()) return -1;
    int bestIdx = 0;
    double bestDist = heuristic(nodes[0], c);
    for (int i = 1; i < (int)nodes.size(); ++i) {
        double d = heuristic(nodes[i], c);
        if (d < bestDist) {
            bestDist = d;
            bestIdx = i;
        }
    }
    return bestIdx;
}

vector<int> uniformCostSearch(
    int start,
    int goal,
    const vector<Coord>& coords,
    const vector<vector<Edge>>& graph,
    double& outTotalCost
) {
    const double INF = numeric_limits<double>::infinity();
    vector<double> dist(coords.size(), INF);
    vector<int> parent(coords.size(), -1);

    using State = pair<double, int>;
    priority_queue<State, vector<State>, greater<State>> pq;

    dist[start] = 0.0;
    pq.push({0.0, start});

    while (!pq.empty()) {
        auto [cost, u] = pq.top();
        pq.pop();

        if (cost > dist[u]) continue;

        if (u == goal) {
            vector<int> path;
            for (int v = goal; v != -1; v = parent[v]) {
                path.push_back(v);
            }
            reverse(path.begin(), path.end());
            outTotalCost = dist[goal];
            return path;
        }

        for (const Edge& e : graph[u]) {
            int v = e.to;
            double newCost = cost + e.dist;
            if (newCost < dist[v]) {
                dist[v] = newCost;
                parent[v] = u;
                pq.push({newCost, v});
            }
        }
    }

    outTotalCost = INF;
    return {};
}

bool parseCsvLine(const string& line,
                  double& lat1, double& lon1,
                  double& lat2, double& lon2,
                  double& dist,
                  string& street,
                  string& roadType)
{
    string token;
    vector<string> tokens;
    stringstream ss(line);

    while (getline(ss, token, ',')) {
        tokens.push_back(token);
    }

    if (tokens.size() < 7) {
        return false;
    }

    try {
        lat1 = stod(tokens[0]);
        lon1 = stod(tokens[1]);
        lat2 = stod(tokens[2]);
        lon2 = stod(tokens[3]);
        dist = stod(tokens[4]);
    } catch (...) {
        return false;
    }

    street = tokens[5];
    if (!street.empty() && street.front() == '"') street.erase(street.begin());
    if (!street.empty() && street.back() == '"') street.pop_back();

    roadType = tokens[6];
    while (!roadType.empty() && isspace((unsigned char)roadType.back()))
        roadType.pop_back();
    while (!roadType.empty() && isspace((unsigned char)roadType.front()))
        roadType.erase(roadType.begin());
    if (!roadType.empty() && roadType.front() == '"') roadType.erase(roadType.begin());
    if (!roadType.empty() && roadType.back() == '"') roadType.pop_back();

    return true;
}

int main() {
    ios::sync_with_stdio(false);

    cout << fixed << setprecision(7);

    string filename;
    cout << "Enter CSV filename (e.g., roads.csv): ";
    cin >> filename;

    ifstream in(filename);
    if (!in) {
        cerr << "Failed to open file: " << filename << "\n";
        return 1;
    }

    unordered_map<Coord, int, CoordHash, CoordEq> coordToIndex;
    vector<Coord> coords;
    vector<vector<Edge>> graph;

    auto getNodeIndex = [&](const Coord& c) {
        auto it = coordToIndex.find(c);
        if (it != coordToIndex.end()) {
            return it->second;
        }
        int idx = (int)coords.size();
        coords.push_back(c);
        coordToIndex[c] = idx;
        graph.emplace_back();
        return idx;
    };

    string line;
    int lineNumber = 0;
    while (getline(in, line)) {
        ++lineNumber;
        if (line.empty()) continue;

        double lat1, lon1, lat2, lon2, dist;
        string street, roadType;
        if (!parseCsvLine(line, lat1, lon1, lat2, lon2, dist, street, roadType)) {
            cerr << "Warning: could not parse line " << lineNumber << "\n";
            continue;
        }

        Coord c1{lat1, lon1};
        Coord c2{lat2, lon2};

        int u = getNodeIndex(c1);
        int v = getNodeIndex(c2);

        graph[u].push_back({v, dist, street, roadType});
        graph[v].push_back({u, dist, street, roadType});
    }

    cout << "Loaded " << coords.size() << " nodes from CSV.\n";

    if (coords.empty()) {
        cerr << "Graph is empty; exiting.\n";
        return 1;
    }

    Coord start, goal;

    cout << "Enter start latitude: ";
    cin >> start.lat;
    cout << "Enter start longitude: ";
    cin >> start.lon;

    cout << "Enter goal latitude: ";
    cin >> goal.lat;
    cout << "Enter goal longitude: ";
    cin >> goal.lon;

    cout << "\nYou entered:\n";
    cout << "  Start: (" << start.lat << ", " << start.lon << ")\n";
    cout << "  Goal : (" << goal.lat << ", " << goal.lon << ")\n";

    int startNode = findClosestNode(coords, start);
    int goalNode = findClosestNode(coords, goal);

    if (startNode == -1 || goalNode == -1) {
        cerr << "Could not find suitable start or goal node.\n";
        return 1;
    }

    cout << "\nClosest node to start: index " << startNode
         << " (" << coords[startNode].lat << ", " << coords[startNode].lon << ")\n";
    cout << "Closest node to goal: index " << goalNode
         << " (" << coords[goalNode].lat << ", " << coords[goalNode].lon << ")\n";

    if (startNode == goalNode) {
        cout << "\nNote: start and goal map to the SAME graph node.\n";
        cout << "Shortest path cost is zero, path is just that node.\n\n";
    }

    double totalCost = 0.0;
    vector<int> path = uniformCostSearch(startNode, goalNode, coords, graph, totalCost);

    if (path.empty()) {
        cout << "No path found using Uniform Cost Search.\n";
        return 0;
    }

    cout << "\nPath found (Uniform Cost Search / Dijkstra):\n";

    for (size_t i = 0; i < path.size(); ++i) {
        int nodeIdx = path[i];
        cout << i << ": (" << coords[nodeIdx].lat << ", "
             << coords[nodeIdx].lon << ")";
        if (i + 1 < path.size()) {
            int nextIdx = path[i + 1];
            for (const Edge& e : graph[nodeIdx]) {
                if (e.to == nextIdx) {
                    cout << " --[" << e.streetName << ", "
                         << e.roadType << ", dist=" << e.dist << "]--> ";
                    break;
                }
            }
        }
        cout << "\n";
    }

    cout << "\nTotal path distance (optimal, sum of edge distances): " << totalCost << "\n";

    return 0;
}