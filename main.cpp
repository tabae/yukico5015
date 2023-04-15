#include <bits/stdc++.h>
#include <sys/time.h>
#include <atcoder/all>

using namespace std;

#define n_kinds 26

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
    vector<vector<int>> d_mod;
    void read() {
        cin >> n >> d >> h;
        s.resize(n);
        for(int i = 0; i < n; i++) cin >> s[i];
        cin >> m;
        sensors.resize(m);
        d_mod.resize(n, vector<int>(n, 1<<30));
        for(auto& [i, j, d] : sensors) {
            cin >> i >> j >> d;
            d_mod[i][j] = d;
        }
        for(int i = 0; i < n; i++) {
            for(int j = 0; j < n; j++) {
                if(s[i][j] == 'S') start_pos = {i, j};
                if(s[i][j] == 'G') goal_pos = {i, j};
                if(s[i][j] == 'K') key_pos = {i, j};
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
    State() : score(1LL<<60) {}
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
        double time_stamp = start_time;
        cerr << "[INFO] - IterationControl::climb - Starts climbing...\n";
        while(time_stamp - start_time + average_time < time_limit) {
            STATE current_state = STATE::generateState(best_state);
            if(current_state.score < best_state.score) {
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
                            // TODO: 厳密な距離に置き換える？ よくよく考えると、宝石や火薬は30％くらいしか無いから、1000^2くらいに収まる
                            //       経路も記録しておけるか？ 火薬は拾いに行く？
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
    vector<pair<int,int>> searchPath(pair<int,int> start, pair<int,int> goal, bool goal_ok) {
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
    long long calcScore(const State& state) {
        long long res = 0;
        return res;
    } 
    Output saveLife(Output raw_output) {
        cerr << "Starts saveLife..." << endl;
        int hp = input.h;
        int turn = 1;
        auto [ci, cj] = input.start_pos;
        bool got_key = false;
        bool got_goal = false;
        Output res;
        for(Operation op: raw_output.ans) {
            if(input.key_pos == make_pair(ci, cj)) {
                got_key = true;
            }
            if(input.goal_pos == make_pair(ci, cj)) {
                got_goal = true;
                break;
            }
            if(got_key) {
                if(hp < 2*input.d*Utils::calc_dist(input.goal_pos, {ci, cj})) break;
            } else {
                if(hp < 2*input.d*Utils::calc_dist(input.key_pos, {ci, cj}) + 3*Utils::calc_dist(input.key_pos, input.goal_pos)) break;
            }
            res.ans.push_back(op);
            if(op.cmd == Command::Move) {
                ci += Utils::di[op.dir];
                cj += Utils::dj[op.dir];
                for(int dir = 0; dir < 4; dir++) {
                    int ni = ci + Utils::di[dir];
                    int nj = cj + Utils::dj[dir];
                    while(ni >= 0 && nj >= 0 && ni < input.n && nj < input.n) {
                        if(input.s[ni][nj] == '#') {
                            break;
                        } else if(input.s[ni][nj] == 'E') {
                            if(turn % input.d_mod[ni][nj] == 0) {
                                hp -= input.d;
                            }
                            break;
                        }
                        ni += Utils::di[dir];
                        nj += Utils::dj[dir];
                    }
                }
            } else {
                assert(false);
            }
            turn++;
            hp--;
        }
        if(!got_goal) {
            vector<pair<int,int>> path;
            if(!got_key) {
                path.push_back({ci, cj});
                ci = input.key_pos.first;
                cj = input.key_pos.second;
            }
            path.push_back({ci, cj});
            path.push_back(input.goal_pos);
            auto ops_to_goal = Utils::convertPathToOperations(path);
            for(auto e : ops_to_goal.ans) res.ans.push_back(e);
        }
        return res;
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
    res.path = Utils::generatePathNN();
    res.score = 0;
    for(int i = 0; i < res.path.size()-1; i++) {
        res.score += Utils::calc_dist(res.path[i], res.path[i+1]);
    }
    return res;
}

State State::generateState(const State &input_state) {
    State res = input_state;
    int i = ryuka.rand(res.path.size()-2)+1;
    int j = ryuka.rand(res.path.size()-2)+1;
    if(i != j) {
        res.score -= Utils::calc_dist(res.path[i-1], res.path[i]);
        res.score -= Utils::calc_dist(res.path[i], res.path[i+1]);
        res.score -= Utils::calc_dist(res.path[j-1], res.path[j]);
        res.score -= Utils::calc_dist(res.path[j], res.path[j+1]);
        swap(res.path[i], res.path[j]);
        res.score += Utils::calc_dist(res.path[i-1], res.path[i]);
        res.score += Utils::calc_dist(res.path[i], res.path[i+1]);
        res.score += Utils::calc_dist(res.path[j-1], res.path[j]);
        res.score += Utils::calc_dist(res.path[j], res.path[j+1]);
    }
    return res;
}   

int main(int argc, char* argv[]) {
    toki.init();
    input.read();
    double temp_start = 95341.24848473762;
    double temp_end = 3.281132005830301;
    #ifdef OPTUNA
    if(argc == 3) {
        temp_start = atof(argv[1]);
        temp_end = atof(argv[2]);
        cerr << "[INFO] - main - temp_start is " << temp_start << "\n";
        cerr << "[INFO] - main - temp_end is " << temp_end << "\n";
    }
    #endif
    IterationControl<State> sera;
    // annealに変更するときは最小値を探していることに気を付ける
    // State res = sera.climb(10, State::initState());
    State res = sera.climb(2.7, State::initState());
    Output ans = Utils::convertPathToOperations(res.path);
    ans = Utils::saveLife(ans);
    ans.print();
}
