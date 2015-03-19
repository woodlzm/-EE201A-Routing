void bfs(const Point* s, const int** G, Path** p, int** len) {
	Point* prev[upperright->x + 2][upperright->y + 2];	//used to backtrace
	int record[upperright->x + 2][upperright->y + 2];	//used to store num of stage
	for (int i = 0; i < upperright->x + 2; ++i)
		for (int j = 0; j < upperright->y + 2; ++j)
			record[i][j] = G[i][j];
    queue<Point*> q;
    record[s->x][s->y] = 0;
    Point* start = new Point(s);
    q.push(start);
	while (!q.empty()) {
		Point* cur = q.front();				//current point to consider
		q.pop();
		int x[4] = {0, 1, 0, -1};
		int y[4] = {1, 0, -1, 0};
		for (int dir = 0; dir < 4; ++dir) {
			int temp = record[cur->x + x[dir]][cur->y + y[dir]];
			if (temp == MAX) {				//this point has not been visited
				q.push(new Point(cur->x + x[dir], cur->y + y[dir]));
				record[cur->x + x[dir]][cur->y + y[dir]] = record[cur->x][cur->y] + 1;
				prev[cur->x + x[dir]][cur->y + y[dir]] = cur;	//record the previous point of the next point
			}
		}
	}
	int now = 0;		//now is the index of Point s in term
	auto itr = terms.begin();
	for (; *itr != s; ++itr, ++now);
	int count = now;		//count is the index of the destination point starting from s
	for (; itr != term.end(); ++itr, ++count) {
		len[now][count] = len[count][now] = record[*itr->x][*itr->y];
		Point* cur = new Point(*itr);
		for (int i = 0; i < record[*itr->x][*itr->y]; ++i) {
			p[now][count].push_back(cur);
			cur = prev[cur->x][cur->y];
		}
		assert(cur->x == s->x && cur->y == s->y);	//for testing
		p[now][count].push_back(cur);
		p[count][now] = p[now][count];
	}
}