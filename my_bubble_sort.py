def bubble_sort(elements):
    n = len(elements)
    for i in range(n - 1):
        for j in range(n - i - 1):
            if(elements[j] < elements[j + 1]):
                elements[j], elements[j + 1] = elements[j + 1], elements[j]
    return elements

if __name__ == '__main__':
    my_list = [2, 10, 5, 8, 9]
    test_elements = bubble_sort(my_list)
    print(test_elements)