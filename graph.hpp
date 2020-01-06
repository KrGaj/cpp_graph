#ifndef GRAPH_HPP
#define GRAPH_HPP

#include <iostream>
#include <optional>
#include <vector>
#include <exception>
#include <type_traits>
#include <queue>
#include <stack>

template <typename V, typename E>
class Graph {
    public:
    class VerticesIterator {
        private:
        std::size_t current_index;
        Graph<V, E>* graph;

        VerticesIterator(Graph<V, E>* rhs, const std::size_t& curr=0u);
        bool isDone() const;

        public:
        VerticesIterator()=delete;
        VerticesIterator(const VerticesIterator& rhs);
        VerticesIterator(VerticesIterator&& rhs) noexcept;
        VerticesIterator& operator=(const VerticesIterator& rhs);
        VerticesIterator& operator=(VerticesIterator&& rhs) noexcept;
        ~VerticesIterator()=default;
        VerticesIterator& operator++();
        VerticesIterator operator++(int);
        bool operator!=(const VerticesIterator& rhs) const;
        bool operator==(const typename Graph<V, E>::VerticesIterator& rhs) const;
        operator bool() const;
        V& operator*() const;
        V* operator->() const;
        VerticesIterator& operator+(const int& add_val);

        friend class Graph<V, E>;
    };

    class EdgesIterator {
        private:
        Graph<V, E>* graph;
        std::size_t current_index1;
        std::size_t current_index2;

        EdgesIterator(Graph<V, E>* rhs_graph, const std::size_t& curr1=0u, const std::size_t& curr2=0u);
        bool isDone() const;

        public:
        EdgesIterator()=delete;
        EdgesIterator(const EdgesIterator& rhs);
        EdgesIterator(EdgesIterator&& rhs) noexcept;
        EdgesIterator& operator=(const EdgesIterator& rhs);
        EdgesIterator& operator=(EdgesIterator&& rhs) noexcept;
        ~EdgesIterator()=default;
        EdgesIterator& operator++();
        EdgesIterator operator++(int);
        bool operator==(const EdgesIterator& rhs) const;
        bool operator!=(const EdgesIterator& rhs) const;
        operator bool() const;
        E& operator*() const;
        E* operator->() const;
        EdgesIterator& operator+(const int& add_val);

        friend class Graph<V, E>;
    };

    class DfsIterator {
        private:
        Graph<V, E>* graph;
        std::stack<std::size_t> vertices_stack;
        std::vector<bool> visited;
        std::size_t current_index;

        DfsIterator(Graph<V, E>* rhs_graph, const std::stack<std::size_t>& rhs_vertices_stack, const std::vector<bool>& rhs_visited, const std::size_t& rhs_current_idx);
        bool isDone() const;

        public:
        DfsIterator()=delete;
        DfsIterator(const DfsIterator& rhs)=default;
        DfsIterator(DfsIterator&& rhs);
        DfsIterator& operator=(const DfsIterator& rhs)=default;
        DfsIterator& operator=(DfsIterator&& rhs);
        ~DfsIterator()=default;

        DfsIterator& operator++();
        DfsIterator operator++(int);
        bool operator==(const DfsIterator& rhs) const;
        bool operator!=(const DfsIterator& rhs) const;
        operator bool() const;
        V& operator*() const;
        DfsIterator& operator+(const int& add_val);

        friend class Graph<V, E>;
    };

    class BfsIterator {
        private:
        Graph<V, E>* graph;
        std::queue<std::size_t> vertices_queue;
        std::vector<bool> visited;
        std::size_t current_index;

        BfsIterator(Graph<V, E>* rhs_graph, const std::queue<std::size_t>& rhs_vertices_queue, const std::vector<bool>& rhs_visited, const std::size_t& rhs_current_idx);
        bool isDone() const;

        public:
        BfsIterator()=delete;
        BfsIterator(const BfsIterator& rhs)=default;
        BfsIterator(BfsIterator&& rhs);
        BfsIterator& operator=(const BfsIterator& rhs)=default;
        BfsIterator& operator=(BfsIterator&& rhs);
        ~BfsIterator()=default;

        BfsIterator& operator++();
        BfsIterator operator++(int);
        bool operator==(const BfsIterator& rhs) const;
        bool operator!=(const BfsIterator& rhs) const;
        operator bool() const;
        V& operator*() const;
        BfsIterator& operator+(const int& add_val);

        friend class Graph<V, E>;
    };

    //private:
    template <typename ITERATOR_TYPE>
    class ConstIterator {
        ITERATOR_TYPE it;

        ConstIterator(const ITERATOR_TYPE& rhs_it);

        public:
        ConstIterator()=delete;
        ConstIterator(const ConstIterator<ITERATOR_TYPE>& rhs)=default;
        ConstIterator(ConstIterator<ITERATOR_TYPE>&& rhs)=default;
        ConstIterator<ITERATOR_TYPE>& operator=(const ConstIterator<ITERATOR_TYPE>& rhs)=default;
        ConstIterator<ITERATOR_TYPE>& operator=(ConstIterator<ITERATOR_TYPE>&& rhs)=default;
        ~ConstIterator()=default;

        ConstIterator<ITERATOR_TYPE>& operator++();
        ConstIterator<ITERATOR_TYPE> operator++(int);
        bool operator==(const ConstIterator<ITERATOR_TYPE>& rhs) const;
        bool operator!=(const ConstIterator<ITERATOR_TYPE>& rhs) const;
        operator bool() const;
        const V& operator*() const;
        ConstIterator<ITERATOR_TYPE>& operator+(const int& add_val);
    };


    private:
    std::vector<V> vertices;    //wierzchołki
    std::vector<std::vector<std::optional<E>>> edges;   //macierz sąsiedztwa
    std::size_t numberOfVertices{0};   //liczba wierzchołków
    std::size_t numberOfEdges{0};  //liczba krawędzi

    V nullVertex{V()};
    E nullEdge{E()};

    public:
    Graph()=default;    //w zupełności wystarczy
    Graph(const Graph<V, E>& rhs)=default;  //konstruktor kopiujący
    Graph(Graph<V, E>&& rhs) noexcept(std::is_nothrow_move_constructible<std::vector<V>>::value 
        && std::is_nothrow_move_constructible<std::vector<std::vector<std::optional<E>>>>::value);   //konstruktor przenoszący

    ~Graph()=default;   //wystarczy - std::vector ma własny destruktor
    Graph<V, E>& operator=(const Graph<V, E>& rhs)=default; //operator kopiujący
    Graph<V, E>& operator=(Graph<V, E>&& rhs) noexcept(std::is_nothrow_move_assignable<std::vector<V>>::value
        && std::is_nothrow_move_assignable<std::vector<std::vector<std::optional<E>>>>::value);  //operator przenoszący

    VerticesIterator insertVertex(const V& vertex_data);    //dodaje wierzchołek
    std::pair<EdgesIterator, bool> insertEdge(std::size_t vertex1_id, std::size_t vertex2_id, const E& label=E(), bool replace=true);
    //dodaje krawędź
    //replace - czy zamieniać (jeśli istnieje) czy dodać drugą krawędź
    bool removeVertex(std::size_t vertex_id);   //usuwa wierzchołek
    bool removeEdge(std::size_t vertex1_id, std::size_t vertex2_id);    //usuwa krawędź
    bool edgeExist(std::size_t vertex1_id, std::size_t vertex2_id) const noexcept;   //sprawdza, czy krawędź istnieje
    std::size_t nrOfVertices() const noexcept;   //zwraca liczbę wierzchołków
    std::size_t nrOfEdges() const noexcept;  //zwraca liczbę krawędzi
    void printNeighborhoodMatrix() const noexcept;   //wypisuje macierz sąsiedztwa

    private:
    VerticesIterator beginVerticesHelper() const;
    VerticesIterator endVerticesHelper() const;
    EdgesIterator beginEdgesHelper() const;
    EdgesIterator endEdgesHelper() const;

    DfsIterator beginDfsHelper(const std::size_t& start_idx) const;
    DfsIterator endDfsHelper() const;
    BfsIterator beginBfsHelper(const std::size_t& start_idx) const;
    BfsIterator endBfsHelper() const;


    public:
    VerticesIterator beginVertices();
    VerticesIterator endVertices();
    EdgesIterator beginEdges();
    EdgesIterator endEdges();
    VerticesIterator begin();
    VerticesIterator end();

    DfsIterator beginDFS(const std::size_t& start_idx);
    DfsIterator endDFS();
    BfsIterator beginBFS(const std::size_t& start_idx);
    BfsIterator endBFS();

    void dfs(std::size_t vertex_idx) const;
    void bfs(std::size_t vertex_idx) const;

    std::pair<double, std::vector<std::size_t>> dijkstra(const std::size_t& start_idx, const std::size_t& end_idx, std::function<double(const E&)> getEdgeLenght=nullptr);
};


/************************************************************************************/

template <typename V, typename E>   //konstruktor przenoszący
Graph<V, E>::Graph(Graph<V, E>&& rhs) noexcept(std::is_nothrow_move_constructible<std::vector<V>>::value 
    && std::is_nothrow_move_constructible<std::vector<std::vector<std::optional<E>>>>::value)
    : vertices(std::move(rhs.vertices)), edges(std::move(rhs.edges)),
    numberOfVertices(rhs.numberOfVertices), numberOfEdges(rhs.numberOfEdges) {

    rhs.numberOfVertices=0;
    rhs.numberOfEdges=0;
}

template <typename V, typename E>   //przenoszący operator przypisania
Graph<V, E>& Graph<V, E>::operator=(Graph<V, E>&& rhs) noexcept(std::is_nothrow_move_assignable<std::vector<V>>::value
        && std::is_nothrow_move_assignable<std::vector<std::vector<std::optional<E>>>>::value) {
    if(this!=&rhs) {
        this->vertices=std::move(rhs.vertices);
        this->edges=std::move(rhs.edges);
        this->numberOfVertices=std::move(rhs.numberOfVertices);
        this->numberOfEdges=std::move(rhs.numberOfEdges);

        rhs.numberOfVertices=0;
        rhs.numberOfEdges=0;
    }

    return *this;
}

template <typename V, typename E>   //dodaje wierzchołek
typename Graph<V, E>::VerticesIterator Graph<V, E>::insertVertex(const V& vertex_data) {
    ++numberOfVertices; //zwiększam liczbę wierzchołków
    vertices.push_back(vertex_data);    //wstawiam nowy wierzchołek na koniec wektora

    edges.resize(numberOfVertices); //zmiana rozmiaru macierzy sąsiedztwa

    for(auto &e : edges) {  //zmiana rozmiaru macierzy sąsiedztwa c. d.
        e.resize(numberOfVertices);
    }

    return VerticesIterator(this, numberOfVertices-1);
}

template <typename V, typename E>   //dodaje krawędź
std::pair<typename Graph<V, E>::EdgesIterator, bool> Graph<V, E>::insertEdge(std::size_t vertex1_id, std::size_t vertex2_id, const E& label, bool replace) {
    if(edges[vertex1_id][vertex2_id].has_value() && !replace) {
        //krawędź już istnieje i nie chcemy jej zastąpić
        return {endEdges(), false};
    }

    if(!edges[vertex1_id][vertex2_id].has_value()) {    //całkowicie nowa krawędź, niezastępująca innej
        ++numberOfEdges;
    }

    edges[vertex1_id][vertex2_id]=std::make_optional<E>(label); //nowa krawędź

    return {EdgesIterator(this, vertex1_id, vertex2_id), true};
}

template <typename V, typename E>   //usuwa wierzchołek
bool Graph<V, E>::removeVertex(std::size_t vertex_id) {
    if(vertices.empty() || vertex_id >= numberOfVertices) {    //brak wierzchołków lub błędny indeks
        return false;
    }

    vertices.erase(vertices.cbegin()+vertex_id);    //usuwam wierzchołek

    auto horizontal_iterator=edges[vertex_id].cbegin(); //iterator idący w poziomie

    for(auto i=0u; i<numberOfVertices; ++i) {
        auto vertical_iterator=edges[i].cbegin()+vertex_id; //iterator idący w pionie, przechodzi w dół

        if((*vertical_iterator).has_value()) {  //iterator wskazuje na istniejącą krawędź
            --numberOfEdges;
        }

        if((*horizontal_iterator).has_value()) {  //iterator wskazuje na istniejącą krawędź
            if(vertical_iterator!=horizontal_iterator) {    //wskazują na ten sam element
                --numberOfEdges;
            }
        }

        ++horizontal_iterator;  //przechodzę w prawo
    }

    for(auto j=0u; j<numberOfVertices; ++j) {
        auto vertical_iterator2=edges[j].cbegin()+vertex_id;
        edges[j].erase(vertical_iterator2); //usuwam komórkę wskazywaną przez "iterator pionowy"
    }

    edges.erase(edges.cbegin()+vertex_id);  //usuwam cały wiersz

    --numberOfVertices; //liczba wierzchołków zmalała

    return true;
}

template <typename V, typename E>   //usuwa krawędź
bool Graph<V, E>::removeEdge(std::size_t vertex1_id, std::size_t vertex2_id) {
    if(!edges[vertex1_id][vertex2_id].has_value() || vertex1_id >= numberOfVertices 
        || vertex2_id >= numberOfVertices) {   //błedny indeks lub nieistniejąca krawędź
        return false;
    }

    edges[vertex1_id][vertex2_id].reset();  //usuwam zawartość std::optional
    --numberOfEdges;

    return true;
}

template <typename V, typename E>   //liczba wierzchołków
std::size_t Graph<V, E>::nrOfVertices() const noexcept {
    return numberOfVertices;
}

template <typename V, typename E>   //liczba krawędzi
std::size_t Graph<V, E>::nrOfEdges() const noexcept {
    return numberOfEdges;
}

template <typename V, typename E>   //czy krawędź istnieje
bool Graph<V, E>::edgeExist(std::size_t vertex1_id, std::size_t vertex2_id) const noexcept {
    if(vertex1_id >= numberOfVertices || vertex2_id >= numberOfVertices) {    //błędny indeks
        return false;
    }

    return (edges[vertex1_id][vertex2_id].has_value());
}

template <typename V, typename E>
void Graph<V, E>::printNeighborhoodMatrix() const noexcept { //wypisuje macierz sąsiedztwa
    for(auto &e : edges) {
        std::cout<<"[ ";

        for(auto &f : e) {
            if(f.has_value()) { //std::optional::has_value() jest moim zdaniem bardziej czytelne
                std::cout<<f.value()<<", ";
            }
            else {
                std::cout<<"(*none*)"<<", ";
            }
        }

        std::cout<<" ]"<<std::endl;
    }
}

template <typename V, typename E>
typename Graph<V, E>::VerticesIterator Graph<V, E>::beginVerticesHelper() const {
    return VerticesIterator(this);
}

template <typename V, typename E>
typename Graph<V, E>::VerticesIterator Graph<V, E>::beginVertices() {
    return beginVerticesHelper();
}

template <typename V, typename E>
typename Graph<V, E>::VerticesIterator Graph<V, E>::endVerticesHelper() const {
    return VerticesIterator(this, numberOfVertices);
}

template <typename V, typename E>
typename Graph<V, E>::VerticesIterator Graph<V, E>::endVertices() {
    return endVerticesHelper();
}

template <typename V, typename E>
typename Graph<V, E>::EdgesIterator Graph<V, E>::beginEdgesHelper() const {
    std::size_t idx1=0u;
    std::size_t idx2=0u;

    while(!edges[idx1][idx2].has_value() && idx1<numberOfVertices) {
        ++idx2;

        if(idx2==numberOfVertices) {
            idx2=0u;
            ++idx1;
        }
    }

    return EdgesIterator(this, idx1, idx2);
}

template <typename V, typename E>
typename Graph<V, E>::EdgesIterator Graph<V, E>::beginEdges() {
    return beginEdgesHelper();
}

template <typename V, typename E>
typename Graph<V, E>::EdgesIterator Graph<V, E>::endEdgesHelper() const {
    return EdgesIterator(this, numberOfVertices);
}

template <typename V, typename E>
typename Graph<V, E>::EdgesIterator Graph<V, E>::endEdges() {
    return endEdgesHelper();
}

template <typename V, typename E>
typename Graph<V, E>::VerticesIterator Graph<V, E>::begin() {
    return beginVerticesHelper();
}

template <typename V, typename E>
typename Graph<V, E>::VerticesIterator Graph<V, E>::end() {
    return endVerticesHelper();
}

template <typename V, typename E>
void Graph<V, E>::dfs(std::size_t vertex_idx) const {
    std::stack<V> vertices_stack;
    std::vector<bool> visited(numberOfVertices, false);

    vertices_stack.push(vertices[vertex_idx]);
    visited[vertex_idx]=true;

    while(!vertices_stack.empty()) {
        auto v=vertices_stack.top();
        vertices_stack.pop();

        std::cout<<v<<", ";

        for(auto i=0u; i<numberOfVertices; ++i) {
            if(edgeExist(vertex_idx, i) && !visited[i]) {
                vertices_stack.push(vertices[i]);
                visited[i]=true;

                vertex_idx=i;

                break;
            }
        }
    }
}

template <typename V, typename E>
void Graph<V, E>::bfs(std::size_t vertex_idx) const {
    std::queue<V> vertices_queue;
    std::vector<bool> visited(numberOfVertices, false);

    vertices_queue.push(vertices[vertex_idx]);
    visited[vertex_idx]=true;

    while(!vertices_queue.empty()) {
        auto v=vertices_queue.front();
        vertices_queue.pop();

        std::cout<<v<<", ";

        for(auto i=0u; i<numberOfVertices; ++i) {
            if(edgeExist(vertex_idx, i) && !visited[i]) {
                vertices_queue.push(vertices[i]);
                visited[i]=true;

                vertex_idx=i;
            }
        }
    }
}

template <typename V, typename E>
typename Graph<V, E>::DfsIterator Graph<V, E>::beginDfsHelper(const std::size_t& start_idx) const {
    if(start_idx>=numberOfVertices) {
        return endDfsHelper();
    }

    std::stack<std::size_t> temp_stack;
    std::vector<bool> temp_visited(numberOfVertices, false);
    temp_stack.push(start_idx);
    temp_visited[start_idx]=true;

    return DfsIterator(this, temp_stack, temp_visited, start_idx);
}

template <typename V, typename E>
typename Graph<V, E>::DfsIterator Graph<V, E>::beginDFS(const std::size_t& start_idx) {
    return beginDfsHelper(start_idx);
}

template <typename V, typename E>
typename Graph<V, E>::DfsIterator Graph<V, E>::endDfsHelper() const {
    std::stack<std::size_t> temp_stack;
    std::vector<bool> temp_visited;

    return DfsIterator(this, temp_stack, temp_visited, numberOfVertices);
}

template <typename V, typename E>
typename Graph<V, E>::DfsIterator Graph<V, E>::endDFS() {
    return endDfsHelper();
}

template <typename V, typename E>
typename Graph<V, E>::BfsIterator Graph<V, E>::beginBfsHelper(const std::size_t& start_idx) const {
    if(start_idx>=numberOfVertices) {
        return endBfsHelper();
    }

    std::queue<std::size_t> temp_queue;
    std::vector<bool> temp_visited(numberOfVertices, false);
    temp_queue.push(start_idx);
    temp_visited[start_idx]=true;

    return BfsIterator(this, temp_queue, temp_visited, start_idx);
}

template <typename V, typename E>
typename Graph<V, E>::BfsIterator Graph<V, E>::beginBFS(const std::size_t& start_idx) {
    return beginBfsHelper(start_idx);
}

template <typename V, typename E>
typename Graph<V, E>::BfsIterator Graph<V, E>::endBfsHelper() const {
    std::queue<std::size_t> temp_queue;
    std::vector<bool> temp_visited;

    return BfsIterator(this, temp_queue, temp_visited, numberOfVertices);
}

template <typename V, typename E>
typename Graph<V, E>::BfsIterator Graph<V, E>::endBFS() {
    return endBfsHelper();
}


/*----->    IMPLEMENTACJA ITERATORA PO WIERZCHOŁKACH    <-----*/


template <typename V, typename E>
Graph<V, E>::VerticesIterator::VerticesIterator(Graph<V, E>* rhs_graph, const size_t& curr) 
    : current_index(curr), graph(rhs_graph) {

}

template <typename V, typename E>
Graph<V, E>::VerticesIterator::VerticesIterator(const VerticesIterator& rhs)
    : current_index(rhs.current_index), graph(rhs.graph) {

}

template <typename V, typename E>
Graph<V, E>::VerticesIterator::VerticesIterator(VerticesIterator&& rhs) noexcept
    : current_index(rhs.current_index), graph(rhs.graph) {
    rhs.current_index=0u;
    rhs.graph=nullptr;
}

template <typename V, typename E>
typename Graph<V, E>::VerticesIterator& Graph<V, E>::VerticesIterator::operator=(const VerticesIterator& rhs) {
    if(this!=&rhs) {
        this->current_index=rhs.current_index;
        this->graph=rhs.graph;
    }

    return *this;
}

template <typename V, typename E>
typename Graph<V, E>::VerticesIterator& Graph<V, E>::VerticesIterator::operator=(VerticesIterator&& rhs) noexcept {
    if(this!=&rhs) {
        this->current_index=rhs.current_index;
        this->graph=rhs.graph;

        rhs.current_index=0u;
        rhs.graph=nullptr;
    }

    return *this;
}

template <typename V, typename E>
typename Graph<V, E>::VerticesIterator& Graph<V, E>::VerticesIterator::operator++() {
    if(!isDone()) {
        ++current_index;
    }

    return *this;
}

template <typename V, typename E>
typename Graph<V, E>::VerticesIterator Graph<V, E>::VerticesIterator::operator++(int) {
    auto old=*this;
    ++(*this);

    return old;
}

template <typename V, typename E>
bool Graph<V, E>::VerticesIterator::operator==(const VerticesIterator& rhs) const {
    return this->graph==rhs.graph && this->current_index == rhs.current_index;
}

template <typename V, typename E>
bool Graph<V, E>::VerticesIterator::operator!=(const VerticesIterator& rhs) const {
    return !(*this==rhs);
}

template <typename V, typename E>
bool Graph<V, E>::VerticesIterator::isDone() const {
    return current_index==graph->vertices.size();    //to chyba powinno być bezpieczniejsze
}

template <typename V, typename E>
Graph<V, E>::VerticesIterator::operator bool() const {
    return !(isDone());
}

template <typename V, typename E>
V& Graph<V, E>::VerticesIterator::operator*() const {
    return isDone() ? graph->nullVertex : graph->vertices[current_index];
}

template <typename V, typename E>
V* Graph<V, E>::VerticesIterator::operator->() const {
    return isDone() ? nullptr : &(graph->vertices[current_index]);
}

template <typename V, typename E>
typename Graph<V, E>::VerticesIterator& Graph<V, E>::VerticesIterator::operator+(const int& add_val) {
    for(int i=0; i<add_val; ++i) {
        ++(*this);
    }

    return *this;
}



/*----->    ITERATOR PO KRAWĘDZIACH     <-----*/



template <typename V, typename E>
Graph<V, E>::EdgesIterator::EdgesIterator(Graph<V, E>* rhs_graph, const std::size_t& curr1, const std::size_t& curr2)
    : graph(rhs_graph), current_index1(curr1), current_index2(curr2) {

}

template <typename V, typename E>
Graph<V, E>::EdgesIterator::EdgesIterator(const EdgesIterator& rhs)
    : graph(rhs.graph), current_index1(rhs.current_index1), current_index2(rhs.current_index2) {

}

template <typename V, typename E>
Graph<V, E>::EdgesIterator::EdgesIterator(EdgesIterator&& rhs) noexcept
    : graph(rhs.graph), current_index1(rhs.current_index1), current_index2(rhs.current_index2) {
        rhs.graph=nullptr;
        rhs.current_index1=0u;
        rhs.current_index2=0u;
}

template <typename V, typename E>
typename Graph<V, E>::EdgesIterator& Graph<V, E>::EdgesIterator::operator=(const EdgesIterator& rhs) {
    if(this!=&rhs) {
        this->graph=rhs.graph;
        this->current_index1=rhs.current_index1;
        this->current_index2=rhs.current_index2;
    }

    return *this;
}

template <typename V, typename E>
typename Graph<V, E>::EdgesIterator& Graph<V, E>::EdgesIterator::operator=(EdgesIterator&& rhs) noexcept {
    if(this!=&rhs) {
        this->graph=rhs.graph;
        this->current_index1=rhs.current_index1;
        this->current_index2=rhs.current_index2;

        rhs.graph=nullptr;
        rhs.current_index1=0u;
        rhs.current_index2=0u;
    }

    return *this;
}

template <typename V, typename E>
typename Graph<V, E>::EdgesIterator& Graph<V, E>::EdgesIterator::operator++() {

    do{
        ++current_index2;

        if(current_index2==graph->nrOfVertices()) {
            current_index2=0u;
            ++current_index1;
        }
    } while(!graph->edgeExist(current_index1, current_index2) && !isDone());

    return *this;
}

template <typename V, typename E>
typename Graph<V, E>::EdgesIterator Graph<V, E>::EdgesIterator::operator++(int) {
    auto old=*this;
    ++(*this);

    return old;
}

template <typename V, typename E>
bool Graph<V, E>::EdgesIterator::operator==(const EdgesIterator& rhs) const {
    return (graph==rhs.graph && current_index1==rhs.current_index1 && current_index2==rhs.current_index2);
}

template <typename V, typename E>
bool Graph<V, E>::EdgesIterator::operator!=(const EdgesIterator& rhs) const {
    return !(*this==rhs);
}

template <typename V, typename E>
bool Graph<V, E>::EdgesIterator::isDone() const {
    return current_index1==graph->nrOfVertices();
}

template <typename V, typename E>
Graph<V, E>::EdgesIterator::operator bool() const {
    return !isDone();
}

template <typename V, typename E>
E& Graph<V, E>::EdgesIterator::operator*() const {
    return isDone() ? graph->nullEdge : graph->edges[current_index1][current_index2].value();
    //zwracam referencję do rzeczywistej wartości lub specjalnie spreparowanego pola grafu,
    //której modyfikacja nie czyni żadnej szkody
    //aczkolwiek późniejsze szukanie błędów może być niemiłym zadaniem
}

template <typename V, typename E>
E* Graph<V, E>::EdgesIterator::operator->() const {
    return isDone() ? nullptr : &(graph->edges[current_index1][current_index2].value());
}

template <typename V, typename E>
typename Graph<V, E>::EdgesIterator& Graph<V, E>::EdgesIterator::operator+(const int& add_val) {
    for(int i=0; i<add_val; ++i) {
        ++(*this);
    }

    return *this;
}



/*----->    ITERATOR DFS    <-----*/



template <typename V, typename E>
Graph<V, E>::DfsIterator::DfsIterator(Graph<V, E>* rhs_graph, const std::stack<std::size_t>& rhs_vertices_stack, const std::vector<bool>& rhs_visited, const std::size_t& rhs_current_idx)
    : graph(rhs_graph), vertices_stack(rhs_vertices_stack), visited(rhs_visited), current_index(rhs_current_idx) {
    
}

template <typename V, typename E>
Graph<V, E>::DfsIterator::DfsIterator(DfsIterator&& rhs) : graph(rhs.graph), vertices_stack(std::move(rhs.vertices_stack)),
    visited(std::move(rhs.visited)), current_index(rhs.current_index) {

    rhs.graph=nullptr;
    rhs.current_index=0;
}

template <typename V, typename E>
typename Graph<V, E>::DfsIterator& Graph<V, E>::DfsIterator::operator=(DfsIterator&& rhs) {

    if(this!=&rhs) {
        this->graph=rhs.graph;
        this->vertices_stack=std::move(rhs.vertices_stack);
        this->visited=std::move(rhs.visited);
        this->current_index=rhs.current_index;

        rhs.graph=nullptr;
        rhs.current_index=0;
    }

    return *this;
}

template <typename V, typename E>
bool Graph<V, E>::DfsIterator::isDone() const {
    return vertices_stack.empty();
}

template <typename V, typename E>
typename Graph<V, E>::DfsIterator& Graph<V, E>::DfsIterator::operator++() {
    vertices_stack.pop();

    for(auto i=0u; i<graph->numberOfVertices; ++i) {
        if(graph->edgeExist(current_index, i) && !visited[i]) {
            vertices_stack.push(i);
            visited[i]=true;
            current_index=i;

            break;
        }
    }
    
    if(isDone()) {
        current_index=graph->numberOfVertices;
    }

    return *this;
}

template <typename V, typename E>
typename Graph<V, E>::DfsIterator Graph<V, E>::DfsIterator::operator++(int) {
    auto old=*this;
    ++(*this);

    return old;
}

template <typename V, typename E>
bool Graph<V, E>::DfsIterator::operator==(const DfsIterator& rhs) const {
    return this->current_index==rhs.current_index;
}

template <typename V, typename E>
bool Graph<V, E>::DfsIterator::operator!=(const DfsIterator& rhs) const {
    return !(*this==rhs);
}

template <typename V, typename E>
Graph<V, E>::DfsIterator::operator bool() const {
    return !isDone();
}

template <typename V, typename E>
V& Graph<V, E>::DfsIterator::operator*() const {
    auto idx=vertices_stack.top();
    return graph->vertices[idx];
}

template <typename V, typename E>
typename Graph<V, E>::DfsIterator& Graph<V, E>::DfsIterator::operator+(const int& add_val) {
    for(int i=0; i<add_val; ++i) {
        ++(*this);
    }

    return *this;
}


/*----->    ITERATOR BFS    <-----*/


template <typename V, typename E>
Graph<V, E>::BfsIterator::BfsIterator(Graph<V, E>* rhs_graph, const std::queue<std::size_t>& rhs_vertices_queue,
    const std::vector<bool>& rhs_visited, const std::size_t& rhs_current_idx)
    : graph(rhs_graph), vertices_queue(rhs_vertices_queue), visited(rhs_visited), current_index(rhs_current_idx) {

}

template <typename V, typename E>
Graph<V, E>::BfsIterator::BfsIterator(BfsIterator&& rhs) : graph(rhs.graph),
    vertices_queue(std::move(rhs.vertices_queue)), visited(std::move(rhs.visited)), current_index(rhs.current_index) {

    rhs.graph=nullptr;
    rhs.current_index=0;
}

template <typename V, typename E>
typename Graph<V, E>::BfsIterator& Graph<V, E>::BfsIterator::operator=(BfsIterator&& rhs) {
    if(this!=&rhs) {
        this->graph=rhs.graph;
        this->vertices_queue=std::move(rhs.vertices_queue);
        this->visited=std::move(rhs.visited);
        this->current_index=rhs.current_index;

        rhs.graph=nullptr;
        rhs.current_index=0;
    }

    return *this;
}

template <typename V, typename E>
bool Graph<V, E>::BfsIterator::isDone() const {
    return vertices_queue.empty();
}

template <typename V, typename E>
typename Graph<V, E>::BfsIterator& Graph<V, E>::BfsIterator::operator++() {
    vertices_queue.pop();

    for(auto i=0u; i<graph->numberOfVertices; ++i) {
        if(graph->edgeExist(current_index, i) && !visited[i]) {
            vertices_queue.push(i);
            visited[i]=true;
            current_index=i;
        }
    }

    if(isDone()) {
        current_index=graph->numberOfVertices;
    }

    return *this;
}

template <typename V, typename E>
typename Graph<V, E>::BfsIterator Graph<V, E>::BfsIterator::operator++(int) {
    auto old=*this;
    ++(*this);

    return old;
}

template <typename V, typename E>
bool Graph<V, E>::BfsIterator::operator==(const BfsIterator& rhs) const {
    return this->current_index==rhs.current_index;
}

template <typename V, typename E>
bool Graph<V, E>::BfsIterator::operator!=(const BfsIterator& rhs) const {
    return !(*this==rhs);
}

template <typename V, typename E>
Graph<V, E>::BfsIterator::operator bool() const {
    return !isDone();
}

template <typename V, typename E>
V& Graph<V, E>::BfsIterator::operator*() const {
    auto idx=vertices_queue.front();
    return graph->vertices[idx];
}

template <typename V, typename E>
typename Graph<V, E>::BfsIterator& Graph<V, E>::BfsIterator::operator+(const int& add_val) {
    for(int i=0; i<add_val; ++i) {
        ++(*this);
    }

    return *this;
}


/*----->    IMPLEMENTACJA CONST-ITERATORA   <-----*/


template <typename V, typename E>
template <typename ITERATOR_TYPE>
Graph<V, E>::ConstIterator<ITERATOR_TYPE>::ConstIterator(const ITERATOR_TYPE& rhs_it) : it(rhs_it) {

}

template <typename V, typename E>
template <typename ITERATOR_TYPE>
typename Graph<V, E>::template ConstIterator<ITERATOR_TYPE>& Graph<V, E>::ConstIterator<ITERATOR_TYPE>::operator++() {
    ++it;

    return *this;
}

template <typename V, typename E>
template <typename ITERATOR_TYPE>
typename Graph<V, E>::template ConstIterator<ITERATOR_TYPE> Graph<V, E>::ConstIterator<ITERATOR_TYPE>::operator++(int) {
    auto old=*this;
    ++(*this);

    return old;
}

template <typename V, typename E>
template <typename ITERATOR_TYPE>
bool Graph<V, E>::ConstIterator<ITERATOR_TYPE>::operator==(const typename Graph<V, E>::template ConstIterator<ITERATOR_TYPE>& rhs) const {
    return this->it==rhs.it;
}

template <typename V, typename E>
template <typename ITERATOR_TYPE>
bool Graph<V, E>::ConstIterator<ITERATOR_TYPE>::operator!=(const typename Graph<V, E>::template ConstIterator<ITERATOR_TYPE>& rhs) const {
    return this->it!=rhs.it;
}

template <typename V, typename E>
template <typename ITERATOR_TYPE>
Graph<V, E>::ConstIterator<ITERATOR_TYPE>::operator bool() const {
    return it;
}

template <typename V, typename E>
template <typename ITERATOR_TYPE>
const V& Graph<V, E>::ConstIterator<ITERATOR_TYPE>::operator*() const {
    return *it;
}

template <typename V, typename E>
template <typename ITERATOR_TYPE>
typename Graph<V, E>::template ConstIterator<ITERATOR_TYPE>& Graph<V, E>::ConstIterator<ITERATOR_TYPE>::operator+(const int& add_val) {
    it=it+add_val;

    return *this;
}

template <typename V, typename E>
std::pair<double, std::vector<std::size_t>> Graph<V, E>::dijkstra(const std::size_t& start_idx, const std::size_t& end_idx, std::function<double(const E&)> getEdgeLenght) {
    
    std::vector<std::size_t> q(nrOfVertices());
    std::vector<std::optional<std::size_t>> predecessors(nrOfVertices());
    std::vector<double> dv(nrOfVertices());
    std::vector<std::size_t> lookingForIndexPredecessors;


    auto comparator=[&dv](const std::size_t& a, const std::size_t& b)->bool {return dv[a] > dv[b];};

    for(auto i=0u; i<nrOfVertices(); ++i) {
        q[i]=i;
        dv[i]=std::numeric_limits<double>::infinity();
    }

    dv[start_idx]=0;
    std::sort(q.begin(), q.end(), comparator);

    try{
        while(!q.empty()) {
            std::size_t u=q.back();
            q.pop_back();

            for(auto i=0u; i<nrOfVertices(); ++i) {
                if(edges[u][i].has_value() && std::find(q.begin(), q.end(), i)!=q.end()) {
                    double path_between_vertices;
                    if(getEdgeLenght!=nullptr) {
                        path_between_vertices=getEdgeLenght(edges[u][i].value());
                    }
                    else {
                        path_between_vertices=edges[u][i].value();
                    }

                    if(dv[i] > dv[u]+path_between_vertices) {
                        dv[i]=dv[u]+path_between_vertices;
                        predecessors[i]=u;
                        std::sort(q.begin(), q.end(), comparator);
                    }
                }
            }
        }
        
        auto idx=end_idx;
        lookingForIndexPredecessors.push_back(idx);
        
        while(idx!=start_idx) {
            lookingForIndexPredecessors.push_back(predecessors[idx].value());
            idx=predecessors[idx].value();
        }

        std::reverse(lookingForIndexPredecessors.begin(), lookingForIndexPredecessors.end());

    } catch(std::bad_optional_access& ex) {
        std::cout<<std::endl;
        std::cout<<"No route from: "<<start_idx<<" to: "<<end_idx<<std::endl;
        std::cout<<std::endl;
        std::exit(1);
    }

    return {dv[end_idx], lookingForIndexPredecessors};
}

#endif  //GRAPH_HPP