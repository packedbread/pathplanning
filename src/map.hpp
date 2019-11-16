#pragma once
#include <algorithm>
#include <cstddef>
#include <ostream>
#include <vector>


namespace planner {
    enum class CellType {
        empty = 0,
        obstacle,
    };

    std::ostream& operator << (std::ostream& out, CellType type);

    struct DefaultMapper {
        template <typename T>
        CellType operator () (const T& value) {
            return value == T{} ? CellType::empty : CellType::obstacle;
        }
    };

    struct Point {
        size_t x;
        size_t y;

        bool operator == (const Point& p) const;
    };

    std::ostream& operator << (std::ostream& out, Point point);

    template <typename CellType = planner::CellType>
    class GridMap {
    public:
        using size_type = typename std::vector<CellType>::size_type;
        using difference_type = typename std::vector<CellType>::difference_type;
        using value_type = typename std::vector<CellType>::value_type;
        using reference = typename std::vector<CellType>::reference;
        using const_reference = typename std::vector<CellType>::const_reference;
        using pointer = typename std::vector<CellType>::pointer;
        using const_pointer = typename std::vector<CellType>::const_pointer;
        using iterator = typename std::vector<CellType>::iterator;
        using const_iterator = typename std::vector<CellType>::const_iterator;
        using reverse_iterator = typename std::vector<CellType>::reverse_iterator;
        using const_reverse_iterator = typename std::vector<CellType>::const_reverse_iterator;

    private:
        size_type width;
        size_type height;
        double cell_size;
        std::vector<value_type> data;
    public:
        GridMap(size_type width, size_type height, double cell_size, const std::vector<value_type>& data) :
            width(width),
            height(height),
            cell_size(cell_size),
            data(data)
        {}

        template <typename T, typename Mapper = planner::DefaultMapper>
        GridMap(size_type width, size_type height, double cell_size, const std::vector<T>& data, Mapper mapper = {}) :
                width(width),
                height(height),
                cell_size(cell_size),
                data(width * height) {
            std::transform(std::begin(data), std::end(data), std::begin(this->data), mapper);
        }

        [[nodiscard]] size_type get_width() const {
            return width;
        }

        [[nodiscard]] size_type get_height() const {
            return height;
        }

        [[nodiscard]] double get_cell_size() const {
            return cell_size;
        }

        value_type operator ()(size_type x, size_type y) const {
            return data[y * width + x];
        }

        iterator begin() {
            return data.begin();
        }
        const_iterator begin() const {
            return data.begin();
        }
        const_iterator cbegin() const {
            return data.cbegin();
        }
        reverse_iterator rbegin() {
            return data.rbegin();
        }
        const_reverse_iterator rbegin() const {
            return data.rbegin();
        }
        const_reverse_iterator crbegin() const {
            return data.crbegin();
        }

        iterator end() {
            return data.end();
        }
        const_iterator end() const {
            return data.end();
        }
        const_iterator cend() const {
            return data.cend();
        }
        reverse_iterator rend() {
            return data.rend();
        }
        const_reverse_iterator rend() const {
            return data.rend();
        }
        const_reverse_iterator crend() const {
            return data.crend();
        }
    };
}
