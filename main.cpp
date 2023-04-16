#include <bits/stdc++.h>
#include <sys/time.h>
#include <atcoder/all>

using namespace std;

double p_erase = 0.75;

/*乱数生成器*/
struct RandGenerator {
    random_device seed_gen;
    mt19937 engine;
    mt19937_64 engine64;
    static const int pshift = 1000000000;
    RandGenerator() : engine(seed_gen()), engine64(seed_gen()) {}
    /*mod以下の乱数を返す（32bit）*/
    int rand(int mod) {
        return engine() % mod;
    }
    /*mod以下の乱数を返す（64bit）*/
    long long randll(long long mod) {
        return engine64() % mod;
    } 
    /*確率pでTrueを返す*/
    bool pjudge(double p) {
        int p_int;
        if(p > 1) p_int = pshift;
        else p_int = p * pshift;
        return rand(pshift) < p_int;
    }
} ryuka;

/*タイマー*/
struct Timer {
    double global_start;
    /*現在の時刻を返す*/
    double gettime() {
        struct timeval tv;
        gettimeofday(&tv, NULL);
        return tv.tv_sec + tv.tv_usec * 1e-6;
    }
    void init() {
        global_start = gettime();
    }
    /*プログラム開始からの経過時間を返す*/
    double elapsed() {
        return gettime() - global_start;
    }
} toki;

struct Input {
    int n, d, h;
    vector<string> s;
    int m;
    vector<tuple<int,int,int>> sensors;
    pair<int,int> start_pos, goal_pos, key_pos;
    vector<vector<bool>> reachable;
    vector<vector<map<pair<int,int>,int>>> dist;
    vector<vector<int>> d_mod, juel_id;
    vector<pair<int,int>> juel_pos;
    int num_juel = 0;
    void read() {
        cin >> n >> d >> h;
        s.resize(n);
        for(int i = 0; i < n; i++) cin >> s[i];
        cin >> m;
        sensors.resize(m);
        d_mod.resize(n, vector<int>(n, 1<<30));
        juel_id.resize(n, vector<int>(n));
        for(auto& [i, j, d] : sensors) {
            cin >> i >> j >> d;
            d_mod[i][j] = d;
        }
        for(int i = 0; i < n; i++) {
            for(int j = 0; j < n; j++) {
                if(s[i][j] == 'S') start_pos = {i, j};
                if(s[i][j] == 'G') goal_pos = {i, j};
                if(s[i][j] == 'K') key_pos = {i, j};
                if(s[i][j] == 'J') {
                    juel_id[i][j] = num_juel;
                    juel_pos.push_back({i, j});
                    num_juel++;
                }
            }
        }
        reachable.resize(n, vector<bool>(n, false));
        queue<pair<int,int>> que;
        que.push(start_pos);
        reachable[start_pos.first][start_pos.second] = true;
        const int di[] = {-1, 1, 0, 0};
        const int dj[] = {0, 0, -1, 1};
        while(!que.empty()) {
            auto [ci, cj] = que.front();
            que.pop();
            for(int k = 0; k < 4; k++) {
                int ni = ci + di[k];
                int nj = cj + dj[k];
                if(ni >= 0 && nj >= 0 && ni < n && nj < n && 
                !reachable[ni][nj] && s[ni][nj] != '#' && s[ni][nj] != 'E' && s[ni][nj] != 'G') {
                    que.push({ni, nj});
                    reachable[ni][nj] = true;
                }
            }
        }
        reachable[goal_pos.first][goal_pos.second] = true;
    }
    bool in_range(int i, int j) {
        if(i >= 0 && j >= 0 && i < n && j < n) return true;
        else return false;
    }
} input;

enum struct Command {
    Stay,
    Move,
    Build,
    Fire
};

struct Operation {
    Command cmd;
    int dir;
    void print();
    Operation() {}
    Operation(Command cmd) : cmd(cmd) {}
    Operation(Command cmd, int dir) : cmd(cmd), dir(dir) {}
};

struct Output {
    vector<Operation> ans;
    void print() {
        for(Operation& e: ans) e.print();
    }
};

struct State {
    long long score;
    vector<pair<int,int>> path;
    vector<bool> juels;
    Output output;
    State() : score(0) {
        juels.resize(input.num_juel, false);
    }
    static State initState(); 
    static State generateState(const State& input_state);
};

/*イテレーション管理クラス*/
template<class STATE>
struct IterationControl {
    int iteration_counter;
    int swap_counter;
    double average_time;
    double start_time;
    IterationControl() : iteration_counter(0), swap_counter(0) {}
    /*山登り法*/
    STATE climb(double time_limit, STATE initial_state) {
        start_time = toki.gettime();
        average_time = 0;
        STATE best_state = initial_state;
        cerr << "[DEBUG] initial score is " << best_state.score << endl;
        double time_stamp = start_time;
        cerr << "[INFO] - IterationControl::climb - Starts climbing...\n";
        while(time_stamp - start_time + average_time < time_limit) {
            STATE current_state = STATE::generateState(best_state);
            if(current_state.score > best_state.score) {
                swap(best_state, current_state);
                swap_counter++;
            }
            iteration_counter++;
            time_stamp = toki.gettime();
            average_time = (time_stamp - start_time) / iteration_counter;
        }
        cerr << "[INFO] - IterationControl::climb - Iterated " << iteration_counter << " times and swapped " << swap_counter << " times.\n";
        return best_state;
    }
    /*焼きなまし法*/
    STATE anneal(double time_limit, double temp_start, double temp_end, STATE initial_state) {
        start_time = toki.gettime();
        average_time = 0;
        STATE best_state = initial_state;
        cerr << "[DEBUG] initial score is " << best_state.score << endl;
        double elapsed_time = 0;
        cerr << "[INFO] - IterationControl::anneal - Starts annealing...\n";
        while(elapsed_time + average_time < time_limit) {
            double normalized_time = elapsed_time / time_limit;
            double temp_current = pow(temp_start, 1.0 - normalized_time) * pow(temp_end, normalized_time);
            STATE current_state = STATE::generateState(best_state);
            long long delta = current_state.score - best_state.score;
            if(delta > 0 || ryuka.pjudge(exp(1.0 * delta / temp_current)) ) {
                swap(best_state, current_state);
                swap_counter++;
            }
            iteration_counter++;
            elapsed_time = toki.gettime() - start_time;
            average_time = elapsed_time / iteration_counter;
        }
        cerr << "[INFO] - IterationControl::anneal - Iterated " << iteration_counter << " times and swapped " << swap_counter << " times.\n";
        return best_state;
    }
};

namespace Utils {
    const int di[] = {-1, 1, 0, 0};
    const int dj[] = {0, 0, -1, 1};
    map<pair<pair<int,int>,pair<int,int>>, vector<pair<int,int>>> path_rec;
    pair<int, int> dir_to_delta(int dir) {
        return {di[dir], dj[dir]};
    }
    pair<int, int> dir_to_pos(int i, int j, int dir) {
        return {i + di[dir], j + dj[dir]};
    }
    char dir_to_UDLR(int dir) {
        return "UDLR"[dir];
    } 
    int calc_dist(pair<int,int> a, pair<int,int> b) {
        return abs(a.first - b.first) + abs(a.second - b.second);
    }
    vector<pair<int,int>> searchPath(pair<int,int> start, pair<int,int> goal, bool goal_ok) {
        if(!goal_ok && path_rec.count({start, goal})) return path_rec[{start, goal}];
        queue<pair<int,int>> que;
        vector seen(input.n, vector<bool>(input.n, false));
        vector from(input.n, vector<pair<int,int>>(input.n));
        que.push(start);
        seen[start.first][start.second] = true;
        while(!que.empty()) {
            auto [ci, cj] = que.front();
            que.pop();
            for(int dir = 0; dir < 4; dir++) {
                int ni = ci + Utils::di[dir];
                int nj = cj + Utils::dj[dir];
                if(ni >= 0 && nj >= 0 && ni < input.n && nj < input.n && !seen[ni][nj] && input.s[ni][nj] != '#' && input.s[ni][nj] != 'E' && !(!goal_ok && input.s[ni][nj] == 'G')) {
                    que.push({ni, nj});
                    seen[ni][nj] = true;
                    from[ni][nj] = {ci, cj};
                }
            }
            if(seen[goal.first][goal.second]) break;
        }
        if(!seen[goal.first][goal.second]) {
            cerr << "Error! (" << start.first << "," << start.second << ") -> ";
            cerr << "(" << goal.first << "," << goal.second << ") is impossible!\n";
            for(auto e : input.s) cerr << e << endl;
            exit(1);
        }
        vector<pair<int,int>> path;
        pair<int,int> current_pos = goal;
        path.push_back(current_pos);
        while(current_pos != start) {
            current_pos = from[current_pos.first][current_pos.second];
            path.push_back(current_pos);
        }   
        reverse(path.begin(), path.end());
        if(!goal_ok) path_rec[{start, goal}] = path;
        return path;
    }
    vector<pair<int,int>> generatePathBFS() {
        cerr << "Starts bfs to key..." << endl;
        vector<pair<int,int>> path2key = searchPath(input.start_pos, input.key_pos, false);
        cerr << "Starts bfs to goal..." << endl;
        vector<pair<int,int>> path2goal = searchPath(input.key_pos, input.goal_pos, true);
        path2key.pop_back();
        vector<pair<int,int>> res;
        for(auto e : path2key) if(input.s[e.first][e.second] != '.') res.push_back(e);
        for(auto e : path2goal) if(input.s[e.first][e.second] != '.') res.push_back(e);
        return res;
    }
    vector<pair<int,int>> generatePathNN() {
        pair<int,int> current_pos = input.start_pos;
        bool got_key = false;
        vector seen(input.n, vector<bool>(input.n, false));
        vector<pair<int,int>> path;
        seen[current_pos.first][current_pos.second] = true;
        while(true) {
            auto [ci, cj] = current_pos;
            path.push_back(current_pos);
            int min_dist = 1<<30;
            pair<int, int> nearest_pos;
            for(int search_range = 5; search_range <= 30; search_range += 5) {
                for(int i = max(0, ci - search_range); i < min(input.n, ci + search_range); i++) {
                    for(int j = max(0, cj - search_range); j < min(input.n, cj + search_range); j++) {
                        if(input.reachable[i][j] && !seen[i][j] && (input.s[i][j] == 'J' || /*input.s[i][j] == 'F' ||*/ input.s[i][j] == 'K')) {
                            int current_dist = abs(i - ci) + abs(j - cj);
                            if(min_dist > current_dist) {
                                min_dist = current_dist;
                                nearest_pos = {i, j};
                            }
                        }
                    }
                }
                if(min_dist != (1<<30)) {
                    current_pos = nearest_pos;
                    seen[nearest_pos.first][nearest_pos.second] = true;
                    if(input.s[nearest_pos.first][nearest_pos.second] == 'K') got_key = true;
                    break;
                }
            }
            if(min_dist == (1<<30)) {
                if(!got_key) path.push_back(input.key_pos);
                path.push_back(input.goal_pos);
                break;
            }
        }
        return path;    
    }
    Output convertPathToOperations(const vector<pair<int,int>>& path) {
        Output res;
        for(int i = 0; i < path.size()-1; i++) {
            auto local_path = searchPath(path[i], path[i+1], i == path.size()-2);
            for(int j = 0; j < local_path.size()-1; j++) {
                pair<int, int> delta = {local_path[j+1].first - local_path[j].first, local_path[j+1].second - local_path[j].second};
                for(int dir = 0; dir < 4; dir++) {
                    if(dir_to_delta(dir) == delta) {
                        res.ans.push_back(Operation(Command::Move, dir));
                        break;
                    }
                } 
            }
        }
        return res;
    }
    long long calcScore(const Output& output) {
        long long score = 0;
        int hp = input.h, mp = 0;
        int turn = 1;
        bool got_key = false;
        bool got_goal = false;
        int invalid_ans = -(1<<30);
        auto [ci, cj] = input.start_pos;
        auto s = input.s;
        for(Operation op: output.ans) {
            if(op.cmd == Command::Move) {
                ci += di[op.dir];
                cj += dj[op.dir];
                hp--;
                if(s[ci][cj] == 'K') got_key = true;
                if(s[ci][cj] == 'G') {
                    if(!got_key || hp < 0) {
                        if(!got_key) cerr << "Warn: invalid ans (goal without key)" << endl;
                        else ; //cerr << "Warn: invalid ans (hp < 0)" << endl;
                        return invalid_ans;
                    } else {
                        return score;
                    }
                }
                if(s[ci][cj] == '#' || s[ci][cj] == 'E') {
                    cerr << "Warn: invalid move, wall or detector" << endl;
                    return invalid_ans;
                }
                if(s[ci][cj] == 'F') mp++; 
                if(s[ci][cj] == 'J') score += 10;
                s[ci][cj] = '.';
                int dmg = 0;
                for(int dir = 0; dir < 4; dir++) {
                    int ni = ci + Utils::di[dir];
                    int nj = cj + Utils::dj[dir];
                    while(ni >= 0 && nj >= 0 && ni < input.n && nj < input.n) {
                        if(input.s[ni][nj] == '#') {
                            break;
                        } else if(input.s[ni][nj] == 'E') {
                            if(turn % input.d_mod[ni][nj] == 0) {
                                dmg += input.d;
                            }
                            break;
                        }
                        ni += Utils::di[dir];
                        nj += Utils::dj[dir];
                    }
                }
                hp -= dmg;
                turn++;
            } else {
                assert(false);
            }
        }
        cerr << "Warn: invalid ans (failed to reach goal)" << endl;
        return invalid_ans;
    } 
}

void Operation::print() {
    if(cmd == Command::Stay) {
        cout << "S\n";
    } else if(cmd == Command::Move) {
        cout << "M " << Utils::dir_to_UDLR(dir) << "\n";
    } else if(cmd == Command::Build) {
        cout << "B " << Utils::dir_to_UDLR(dir) << "\n";
    } else if(cmd == Command::Fire) {
        cout << "F " << Utils::dir_to_UDLR(dir) << "\n";
    }
}

State State::initState() {
    State res;
    res.path = Utils::generatePathBFS();
    for(auto e : res.path) {
        if(input.s[e.first][e.second] == 'J') {
            res.juels[input.juel_id[e.first][e.second]] = true;
        }
    }
    res.output = Utils::convertPathToOperations(res.path);
    res.score = Utils::calcScore(res.output);
    return res;
}

State State::generateState(const State &input_state) {
    State res = input_state;
    if(res.path.size() >= 20 && ryuka.pjudge(p_erase)) {
        int rm_target = -1;
        while(true) {
            int tmp = ryuka.rand(res.path.size() - 2) + 1;
            if(input.s[res.path[tmp].first][res.path[tmp].second] != 'K') {
                rm_target = tmp;
                break;
            }
        } 
        if(rm_target != -1) {
            res.juels[input.juel_id[res.path[rm_target].first][res.path[rm_target].second]] = false;
            res.path.erase(res.path.begin() + rm_target);
        }
    }
    int target = -1;
    for(int _ = 0; _ < 100; _++) {
        int tmp = ryuka.rand(input.num_juel);
        if(!res.juels[tmp] && input.reachable[input.juel_pos[tmp].first][input.juel_pos[tmp].second]) {
            target = tmp;
            break;
        }
    }
    if(target == -1) {
        cerr << "Warn: cannot find juel candidate" << endl;
        return res;
    }
    int min_dist = 1<<30, min_pos = -1;
    for(int i = 0; i < res.path.size()-1; i++) {
        int tmp_dist1 = Utils::calc_dist(input.juel_pos[target], res.path[i]);
        int tmp_dist2 = Utils::calc_dist(input.juel_pos[target], res.path[i+1]);
        int tmp_dist = tmp_dist1 + tmp_dist2;
        if(tmp_dist < min_dist) {
            min_pos = i;
            min_dist= tmp_dist;
        }
    }
    if(min_pos == -1) {
        cerr << "Warn: cannot find insertable pos" << endl;
        return res;
    }
    res.path.insert(res.path.begin()+min_pos+1, input.juel_pos[target]);
    res.juels[target] = true; 
    res.output = Utils::convertPathToOperations(res.path);
    res.score = Utils::calcScore(res.output);
    return res;
}   

int main(int argc, char* argv[]) {
    toki.init();
    input.read();
    #ifdef OPTUNA
    p_erase = atof(argv[1]);
    p_ramdom_add = atof(argv[2]);
    #endif
    IterationControl<State> sera;
    State res = sera.climb(2.8, State::initState());
    res.output.print();
}
