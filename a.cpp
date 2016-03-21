#include "route.h"
#include "lib_record.h"
#include <stdio.h>
#include <iostream>
#include <cstdlib>
#include <cstring>
#include <ctime>
#include <algorithm>
//constant
const int MAXV = 600 + 5;
const int MAXE = 600 * 8 + 5;

//variable
int S, T;	//start and end vertice
int edge_num;
int demand[MAXV];
int Stime;
//function
void loadData(char *topo[5000], int edge_num, char *demand);
void output_ans(int *a);
int checkConnectivity();
//
struct _edge
{
    int id;
    int u, v, w;
}edge[MAXE];
int adj[MAXV][10];
void line(int id, int u, int v)
{
    adj[u][++adj[u][0]] = id;
}
int dist[MAXV][MAXV];
int vis[MAXV];
int ans[MAXE], score = 0x3f3f3f3f;
int tmp_ans[MAXE];
int disT[MAXV];
int lenDemand;
int minJump[MAXV];
int maxLink[MAXV];
bool done;
void dfs(int u, int tmp_score)
{
    //if(clock() > 9 * 1000000) return;
    //std::cerr << u << ' ' << tmp_score << ' ' << lenDemand << std::endl;
    if(score <= tmp_score + lenDemand)
    {
        done = 0;
        return;
    }
    //std::cerr << u << ' ' << tmp_score << ' ' << lenDemand << std::endl;
    //if(100 <= tmp_score) return;
    if(u == T)
    {
        //std::cerr << tmp_score << ' ' << tmp_ans[0] << std::endl;
        if(score > tmp_score)
        if(demand[0] == 0)
        {
            std::cout << clock() / 1e6 << ' ' << tmp_score << std::endl;

            score = tmp_score;
            for(int i = 0; i <= tmp_ans[0]; ++i)
                ans[i] = tmp_ans[i];
        }
        return;
    }
    if(vis[u]) return;
    vis[u] = 1;
    if(demand[u] == 1)
    {
        lenDemand -= minJump[u];
    }
    for(int i = 1; i <= adj[u][0]; ++i)
    {
        int index = i;
        int id, v, w;
        id = adj[u][index];
        v = edge[adj[u][index]].v;
        w = edge[adj[u][index]].w;
        if(demand[v] == 1)
        {
            --demand[0];
        }
        tmp_ans[++tmp_ans[0]] = id;
        //
        dfs(v, tmp_score + w);

        if(demand[v] == 1)
        {
            ++demand[0];
            //lenDemand += edge[adj[v][1]].w;
        }
        --tmp_ans[0];
    }
    if(demand[u] == 1)
    {
        lenDemand += minJump[u];
    }
    vis[u] = 0;
}
namespace gedian
{
    int cnt = 0;
    int idx[MAXV];
    int low[MAXV];
    bool vis[MAXV];
    int ret[MAXV];
    bool done = 0;
    int st[MAXV];
    void dfs(int u, int fa)
    {
        if(vis[u]) return;
        if(u == T)
        {
            for(int i = 1; i <= st[0]; ++i)
            {
                ret[++ret[0]] = st[i];
            }
        }
        vis[u] = 1;
        st[++st[0]] = u;
        idx[u] = low[u] = ++cnt;
        int v;
        for (int i = 1; i <= adj[u][0]; ++i)
        {
            v = edge[adj[u][i]].v;
            dfs(v, u);
            if(v != fa)
            {
                low[u] = std::min(low[u], low[v]);
            }
        }

        --st[0];
    }
    int checkLink(int S, int T, int ban)
    {
        //std::cerr << S << ' ' << T << ' ' << ban << std::endl;
        bool vis[MAXV] = {0};
        vis[ban] = 1;
        int st[MAXV];
        int l = 0, r = 0;
        vis[st[r++] = S] = 1;
        int u, v;
        while(l < r)
        {
            u = st[l++];
            if(u == T) return 1;
            for (int i = 1; i <= adj[u][0]; ++i)
            {
                v = edge[adj[u][i]].v;
                //if(v == T) return 1;
                if(!vis[v])
                    vis[st[r++] = v] = 1;
            }
        }
        return 0;
    }
    void solve()
    {
        memset(vis, 0, sizeof(vis));
        //dfs(S, 0);
        //for (int i = 1; i <= 100; ++i)
        //if(idx[i] && low[i] == idx[i])
        //{
        //	std::cerr << "ge : " << i << std::endl;
        //}
        std::cerr << std::endl;
        for (int i = 1; i <= 50; ++i)
        {
            if(!checkLink(S, T, i))
                std::cerr << "GG : " << i << std::endl;
        }

        //for (int i = 1; i <= ret[0]; ++i)
        //if(idx[st[i]] && low[st[i]] == idx[st[i]])
        //{
        //	std::cerr << st[i] << ' ';
        //}
        //std::cerr << std::endl;
    }
}//namespace gedian
void init()
{
    for (int i = 1; i < 600; ++i)
        if(demand[i])
            lenDemand += minJump[i];
    gedian::solve();
}
//你要完成的功能总入口
void search_route(char *topo[5000], int edge_num, char *demand)
{
    loadData(topo, edge_num, demand);
    init();
    //if(checkConnectivity())
    if(lenDemand < 0x3f3f3f3f)
    {
        for(score = lenDemand - 19; ans[0] == 0; )
        {
            done = 1;
            score += 20;
            std::cout << "limit score : " << score << std::endl;
            dfs(S, 0);
            //dfs(6, 1);
            if(done)
            {
                score = -1;
                break;
            }
        }
    }
    else
    {
        score = -1;
    }

    output_ans(ans);
}

////
int checkConnectivity()
{
    bool vis[MAXV] = {0};
    int st[MAXV];
    int l = 0, r = 0;
    int u, v;
    vis[st[r++] = S] = 1;
    while(l < r)
    {
        u = st[l++];
        for (int i = 1; i <= adj[u][0]; ++i)
        {
            v = edge[adj[u][i]].v;
            if(v == T) return 1;
            if(!vis[v])
                vis[st[r++] = v] = 1;
        }
    }
    return 0;
}
int Str2Int(char* &str)
{
    while(str[0] && !('0' <= str[0] && str[0] <= '9'))
        ++str;
    if(str[0] == 0) return -1;
    int ret = 0;
    while('0' <= str[0] && str[0] <= '9')
    {
        ret = ret * 10 + str[0] - '0';
        ++str;
    }
    ++str;
    return ret;
}
bool comp_w(int x, int y)
{
    return edge[x].w < edge[y].w;
}
void loadData(char *topo[5000], int _edge_num, char *_demand)
{
    edge_num = _edge_num;

    int t[MAXV][MAXV] = {0};
    int id, x, y, z;
    for (int i = 0; i < edge_num; ++i)
    {
        sscanf(topo[i], "%d,%d,%d,%d", &id, &x, &y, &z);
        ++id, ++x, ++y;
        edge[id].id = id;
        edge[id].u = x;
        edge[id].v = y;
        edge[id].w = z;
        //line(id, x, y);
        if(t[x][y] == 0)
        {
            t[x][y] = id;
        }
        else
        {
            if(edge[t[x][y]].w > z)
            {
                t[x][y] = id;
            }
        }
    }
    S = Str2Int(_demand) + 1;
    T = Str2Int(_demand) + 1;
    while((x = Str2Int(_demand)) != -1)
    {
        demand[0]++;
        demand[x + 1] = 1;
    }

    for (int i = 1; i <= 100; ++i)
        for (int j = 1; j <= 100; ++j)
            if(i == j)
                dist[i][j] = 0;
            else
                dist[i][j] = 0x3f3f3f3f;
    for (int i = 1; i <= 100; ++i)
        for (int j = 1; j <= 100; ++j)
            if(i != j && t[i][j])
            if(i != T && j != S)
            {
                line(t[i][j], i, j);
                dist[i][j] = edge[t[i][j]].w;
            }

    for (int k = 1; k <= 100; ++k)
        for (int i = 1; i <= 100; ++i)
            for (int j = 1; j <= 100; ++j)
                dist[i][j] = std::min(dist[i][j], dist[i][k] + dist[k][j]);

    for (int i = 1; i <= 100; ++i)
        if(demand[i])
        {
            minJump[i] = 0x3f3f3f3f;
            for (int j = 1; j <= 100; ++j)
                if(i != j)
                if(demand[j] || j == T)
                {
                    minJump[i] = std::min(minJump[i], dist[i][j]);
                }
        }

    for (int i = 0; i < 600; ++i)
        std::sort(&adj[i][1], &adj[i][1] + adj[i][0], comp_w);
}
void output_ans(int *a)
{
    std::cerr << "ans = " << score << std::endl;
    for (int i = 1; i <= a[0]; ++i)
    {
        record_result(a[i] - 1);
        if(i > 1)
            std::cerr << "|";
        std::cerr << a[i] - 1;
    }
    std::cerr << std::endl;

    for (int i = 1, u = S; i <= a[0]; ++i)
    {
        std::cerr << u << "->";
        u = edge[a[i]].v;
    }
    std::cerr << T << std::endl;
}
