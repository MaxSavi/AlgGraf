using System;
using System.Collections.Generic;

public class Graf
{
    List<string> counter = new List<string>();
    private Dictionary<string, List<string>> graph = new Dictionary<string, List<string>>();

    public void AddNode(string node)
    {
        if (!graph.ContainsKey(node))
        {
            graph.Add(node, new List<string>());
            counter.Add(node);
        }
    }

    public void AddEdge(string node1, string node2)
    {
        if (graph.ContainsKey(node1) && graph.ContainsKey(node2) && !graph[node1].Contains(node2) && !graph[node2].Contains(node1))
        {
            graph[node1].Add(node2);
            graph[node2].Add(node1);
        }
    }

    public void DeleteNode(string node)
    {
        if (graph.ContainsKey(node))
        {
            List<string> nodes = graph[node];
            graph.Remove(node);
            foreach (string nod in nodes)
            {
                graph[nod].Remove(node);
            }
        }
    }

    public void DeleteEdge(string node1, string node2)
    {
        if (graph.ContainsKey(node1) && graph.ContainsKey(node2))
        {
            graph[node1].Remove(node2);
            graph[node2].Remove(node1);
        }
    }

    public void VisualizeGraph()
    {
        Console.WriteLine("Graph Visualization:");
        foreach (var startVertex in graph.Keys)
        {
            foreach (var endVertex in graph[startVertex])
            {
                Console.WriteLine($"{startVertex} --- {endVertex}");
            }
        }
    }
    public void dfs(string node, HashSet<string> visited = null, int[] distance = null)
    {
        string nextnode;
        if (visited == null)
        {
            visited = new HashSet<string>();
        }
        if (distance == null)
        {
            distance = new int[counter.Count];
        }
        visited.Add(node);
        foreach(var NextNode in graph[node])
        {
            if (!visited.Contains(NextNode))
            {
                distance[counter.IndexOf(NextNode)] = distance[counter.IndexOf(node)] + 1;
                Console.WriteLine($"Текущая нода: {node}");
                Console.WriteLine(string.Join(", ", distance));
                dfs(NextNode, visited, distance);
            }
        }
    }
    public void bfs(string node)
    {
        int[] distance = new int[counter.Count];
        string curr_node;
        HashSet<string> visited_bfs = new HashSet<string>();
        Queue<string> queue = new Queue<string>();
        visited_bfs.Add(node);
        queue.Enqueue(node);
        while(queue.Count > 0)
        {
            curr_node = queue.Dequeue();
            foreach(var NextNode in graph[curr_node])
            {
                if (!visited_bfs.Contains(NextNode))
                {
                    queue.Enqueue(NextNode);
                    visited_bfs.Add(NextNode);
                    distance[counter.IndexOf(NextNode)] = distance[counter.IndexOf(curr_node)] + 1;
                    Console.WriteLine($"Текущая нода: {curr_node}, Следующая нода: {NextNode}");
                    Console.WriteLine(string.Join(", ", distance));
                }
            }
        }
    }
    public int[,] AdjacencyMatrixCalc()
    {
        int[,] adjacency = new int[counter.Count, counter.Count];
        for (int i = 0, n = counter.Count; i < n; i++)
        {
            List<string> listOfStr = graph[counter[i]];
            for (int j = 0; j < listOfStr.Count; j++)
            {
                int indexToChange = counter.IndexOf(listOfStr[j]);
                adjacency[i, indexToChange] = 1;
            }
        }
        return adjacency;
    }
    public static void Main(string[] args)
    {
        Graf graf = new Graf();
        graf.AddNode("MIT");
        graf.AddNode("Park");
        graf.AddNode("Boylston");
        graf.AddNode("Tufts");
        graf.AddNode("Gov");
        graf.AddNode("Hay");
        graf.AddNode("State");
        graf.AddNode("Downtown");
        graf.AddNode("China Town");
        graf.AddNode("South");
        graf.AddNode("Airport");
        graf.AddEdge("MIT", "Park");
        graf.AddEdge("Boylston", "Park");
        graf.AddEdge("Downtown","Park");
        graf.AddEdge("Gov", "Park");
        graf.AddEdge("Boylston", "Tufts");
        graf.AddEdge("Boylston", "Downtown");
        graf.AddEdge("Tufts", "China Town");
        graf.AddEdge("Tufts", "South");
        graf.AddEdge("Gov","Hay");
        graf.AddEdge("Gov", "State");
        graf.AddEdge("State", "Hay");
        graf.AddEdge("State", "Downtown");
        graf.AddEdge("China Town", "Downtown");
        graf.AddEdge("South", "Downtown");
        graf.AddEdge("South", "China Town");
        graf.AddEdge("South", "Airport");
        graf.AddEdge("State", "Airport");
        graf.VisualizeGraph();
        Console.WriteLine("------------<DFS>------------");
        graf.dfs("MIT");
        Console.WriteLine("------------<BFS>------------");
        graf.bfs("MIT");
        Console.WriteLine("------------<Матрица смежности>------------");
        var arr = graf.AdjacencyMatrixCalc();
        for (int i = 0; i < arr.GetLength(0); i++)
        {
            for (int j = 0; j < arr.GetLength(1); j++)
            {
                Console.Write(arr[i, j] + " ");
            }
            Console.WriteLine();
        }
    }
}