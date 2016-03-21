
#include "route.h"
#include "lib_record.h"
#include <algorithm>
#include <vector>
#include <iostream>
#include <cstring>
#include <stdio.h>
#include <sstream>
#include <queue>
#include <unordered_map>
#include <map>
#include <__hash_table>
using namespace std;
#define DEBUG_SHOW_PATH
//#define DEBUG_SHOW_STFN
//#define DEBUG_SHOW_DIST
//你要完成的功能总入口
const int max_n = 660;

int max_vertexnum = 0;
const int inf = 0x3f3f3f3f;
struct Edge{
    int to,num,cost;
    Edge(){}
    Edge(int _to,int _cost,int _num){
        to = _to;
        num = _num;
        cost = _cost;
    }
    bool operator<(const Edge& a)const{
        return cost<a.cost;
    }
};
vector<Edge> graph[max_n];
int mapp[max_n][max_n];
int dist[max_n][max_n];
vector<int> vertexes;
vector<int> result;
pair<int,int> stfn;
bool visit[max_n];
vector<int> resPath;
struct pairHash{
    template <typename T1,typename T2>
            size_t operator()(const std::pair<T1,T2> &pr)const {
        using std::hash;
        return hash<T1>()(pr.first)^hash<T2>()(pr.second);
    }
};
unordered_map<pair<int,int>,vector<int>,pairHash > path; //init the unorder_map
unordered_map<pair<int,int>,int,pairHash> hashPath;
void printPath(const pair<int,int> p);
void solve_input_topo(char *topo[5000],int edge_num){
    /*
     * solve the case that there are more than two edge between two vertexes*/
    memset(mapp,inf, sizeof(mapp));
    memset(dist,inf, sizeof(dist));
    hashPath.clear();
    path.clear();
    int tmp[4];
    for(int i = 0;i<max_n;i++){
        graph[i].clear();
    }
    for(int i = 0;i<edge_num;i++){
        int elenum = 0;
        int pos = 0;
        string num;
        while (elenum<4){
            num.clear();
            while (isdigit(topo[i][pos])){
                num+=topo[i][pos];
                pos++;
            }
            tmp[elenum++] = stoi(num);
            pos++;
        }
        graph[tmp[1]].push_back(Edge(tmp[2],tmp[3],tmp[0]));


        path[make_pair(tmp[1],tmp[2])].push_back(tmp[0]);
        hashPath[make_pair(tmp[1],tmp[2])] = tmp[0];
#ifdef DEBUG_SHOW_PATH

       // cout<<path[make_pair(tmp[1],tmp[2])].size()<<" size"<<endl;
        printPath(make_pair(tmp[1],tmp[2]));
        //cout<<"vector  "<<path[make_pair(tmp[1],tmp[2])][0]<<" "<<endl;
       // cout<<tmp[0]<<"--> num"<<endl;
#endif
        mapp[tmp[1]][tmp[2]] = tmp[3];
        dist[tmp[1]][tmp[2]] = tmp[3];
        max_vertexnum = max(max_vertexnum,tmp[2]);
        max_vertexnum = max(max_vertexnum,tmp[1]);

#ifdef DEBUG_SHOW_DIST
        for(int j = 0;j<4;j++){
            cout<<tmp[j]<<" ";
        }
        cout<<'\n';
#endif

    }
    max_vertexnum=max_vertexnum+1;
}
void solve_input_demand(char *demand){
    vertexes.clear();
    int tmp[4];
    int demand_len = strlen(demand);//the type of demand_len is size_t
    int denum = 0;

    for(int i=0;i<demand_len;i++){
        string num;
        num.clear();
        while (isdigit(demand[i])){
            num+=demand[i];
            i++;
        }
        if(denum<2){
            tmp[denum] = stoi(num);
            denum++;
        } else{
            vertexes.push_back(stoi(num));
        }
    }

    stfn.first = tmp[0];
    stfn.second = tmp[1];
#ifdef DEBUG_SHOW_STFN
    cout<<"st "<<stfn.first<<" fn "<<stfn.second<<endl;
    for(vector<int>::iterator it=vertexes.begin();it!=vertexes.end();++it){
        cout<<*it<<" ";
    }
    cout<<"\n";
#endif
}
void printMap(int n){
    for(int i = 0;i<n;i++){
        for(int j = 0;j<n;j++){
            cout<<dist[i][j]<<" ";
        }
        cout<<'\n';
    }
}

void printPath(const pair<int,int> p);

void printPath(const pair<int,int> p) {
    cout<<"from "<<p.first<<" to "<<p.second<<" "<<path[p].size()<<endl;
    for(vector<int>::iterator it = path[p].begin();it!=path[p].end();++it){
        cout<<*it<<" ";
    }
    cout<<'\n';
}

void unionPath(const pair<int,int> a,const pair<int,int> b){
    pair<int,int> res = make_pair(a.first,b.second);
    path[res].reserve(path[a].size()+path[b].size());
    path[res].assign(path[a].begin(),path[a].end());
    //printPath(res);
    path[res].insert(path[res].end(),path[b].begin(),path[b].end());
    //printPath(res);
}
//find the shortest path in set vs obtain source vertex
void prim(const vector<Edge>graph[],vector<int> vs,int n){
    path.clear();
    for(int i = 0;i<n;i++){
       dist[i][i] = 0;
    }
    for(int k = 0;k<n;k++){
        if(k==stfn.first||k==stfn.second){
            continue;
        }
        for(int i = 0;i<vs.size();i++){
            if(vs[i]==stfn.second){
                continue;
            }
            for(int j = 0;j<vs.size();j++){
                int from = vs[i];
                int to = vs[j];
                if(dist[from][k]+dist[k][to]<dist[from][to]){
                    dist[from][to] = dist[from][k]+dist[k][to];
                    //path<make_pair<i,j>,vector<int> > = path<make_pair<i,k>,vector<int> > + path<make_pair<i,k>, vector<int> >
                    //if the sourceID and the destinationID in the path of i and j solve it
                    /*
                     * coding
                     * */
                    unionPath(make_pair(from,k),make_pair(k,to));
                  //  printPath(make_pair(from,to));

                }
            }
        }
    }



}

void shortest_path(const vector<Edge>graph[],vector<int> vs,int n)
{
  //  path.clear();
    for(int i = 0;i<n;i++){
        dist[i][i] = 0;
    }
    bool isVs[max_n];
    vs.push_back(stfn.first);
    memset(isVs,0, sizeof(isVs));
    for(vector<int>::iterator it = vs.begin();it!=vs.end();++it){
        isVs[*it] = true;
    }
    cout<<isVs[stfn.first]<<" "<<isVs[stfn.second]<<endl;
    for(int k = 0;k<n;k++){
        if(k==stfn.first||k==stfn.second||!isVs[k]){
            continue;
        }
        for(int i = 0;i<vs.size();i++){
            for(int j = 0;j<vs.size();j++){
                if(vs[j] == stfn.first){
                    continue;
                }
                int from = vs[i];
                int to = vs[j];
                if(dist[from][k]+dist[k][to]<dist[from][to]){
                    dist[from][to] = dist[from][k]+dist[k][to];
                    //path<make_pair<i,j>,vector<int> > = path<make_pair<i,k>,vector<int> > + path<make_pair<i,k>, vector<int> >
                    //if the sourceID and the destinationID in the path of i and j solve it
                    /*
                     * coding
                     * */
                    unionPath(make_pair(from,k),make_pair(k,to));
                    //printPath(make_pair(from,to));

                }
            }
        }
    }

}
int mincost = inf;

void printVector(vector<int> s);
void dfs(int s,int cost,int num){
    if(num == vertexes.size()){
        if(dist[s][stfn.second]<inf&&cost+dist[s][stfn.second]<mincost){

            resPath.push_back(hashPath[make_pair(s,stfn.second)]);
            cout<<"______________________________________"<<endl;
            cout<<"cost "<<cost+dist[s][stfn.second]<<endl;

            mincost = cost + dist[s][stfn.second];
            for(int i = 0;i<resPath.size();i++){
                cout<<resPath[i]<<" ";
            }
            cout<<'\n';
            resPath.pop_back();
        }
        return;
    } else{
        for(vector<int>::iterator it = vertexes.begin();it!=vertexes.end();++it){
            if(!visit[*it]){
                if(dist[s][*it]<inf){
                    visit[*it] = true;
                    cost += dist[s][*it];
                    resPath.insert(resPath.end(),path[make_pair(s,*it)].begin(),path[make_pair(s,*it)].end());
                    printVector(resPath);
                    dfs(*it,cost,num+1);
                    for(int i = 0;i<path[make_pair(s,*it)].size();i++){
                        resPath.pop_back();
                    }
                    cost -= dist[s][*it];
                    visit[*it] = false;
                }
            }
        }
    }
}
void dijkstra(const vector<Edge> graph[],int s){
    int di[max_n];
    priority_queue<Edge> qu;
    int num_edge = -1;
    qu.push(Edge(s,0,num_edge));
    while (!qu.empty()){
        Edge now = qu.top();
        qu.pop();
        int now_vertex = now.to;
        if(visit[now_vertex]){
            continue;
        }
        visit[now_vertex] = true;
        int min_cost = now.cost;
        for(int i = 0;i<graph[now_vertex].size();i++){
            if(!visit[graph[now_vertex][i].to]&&graph[now_vertex][i].to!=stfn.second
               &&graph[now_vertex][i].cost+min_cost<di[graph[now_vertex][i].to]){
                pair<int,int> tmp;
                pair<int,int> ori;
                //copy path from ori to tmp, and add edge to the tmp path
                tmp = make_pair(s,graph[now_vertex][i].to);
                ori = make_pair(s,now_vertex);
                copy(path[ori].begin(),path[ori].end(),back_inserter(path[tmp]));
                path[tmp].push_back(graph[now_vertex][i].num);
                di[graph[now_vertex][i].to] = graph[now_vertex][i].cost+min_cost;
                //
                //push the new vertex into the priority_queue
                qu.push(Edge(graph[now_vertex][i].to,di[graph[now_vertex][i].to],num_edge));
            }

        }

    }

}
void solve_search(){

    /*
     * 1) the calculation of the shortest path (without restrictions) between every node pair belonging to VS,
     * and also between the source node s and every node in VS;
     * 2) the calculation of the shortest path from every node in VS to the target node.
     * Revision of the algorithm by Saksena and Kumar
     * */
    memset(visit,0, sizeof(visit));
    result.clear();
    //prim(graph,vertexes,max_vertexnum);
    resPath.clear();
    shortest_path(graph,vertexes,max_vertexnum);
    memset(visit,0,sizeof(visit));
    dfs(stfn.first,0,0);

}
void branchAndBound(){
    /*use branch And Bound to solve this problem
     * just similar to  tsp
     * the shortest path between two vertexes in the must include set
     * is the path that exclude any other vertexes which is in the
     * must include set.
     * this algorithm based on the constraints in a given node,
     * a lower bound is formulated for the given node
     * construction of solution tree*/
}
void test(){
    unionPath(make_pair(2,3),make_pair(3,1));
    printPath(make_pair(2,1));
}


void printVector(vector<int> s) {
    for(vector<int>::iterator it = s.begin();it!=s.end();++it){
        cout<<*it<<" -> ";
    }
    cout<<'\n';
}

void search_route(char *topo[5000], int edge_num, char *demand) {
    /*unsigned short result[] = {2, 6, 3};//示例中的一个解
    for (int i = 0; i < 3; i++)
        record_result(result[i]);*/
    solve_input_topo(topo,edge_num);
    solve_input_demand(demand);
    //printMap(max_vertexnum);
    solve_search();
    //test();


}
