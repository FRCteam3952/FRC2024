package frc.robot.util;

public class WrappingBuffer<T> {
    private Object[] arr;
    private int start = 0;
    private int size = 0;

    public WrappingBuffer(int size) {
        this.arr = new Object[size];
    }

    @SuppressWarnings("unchecked")
    public T get(int idx) {
        if(idx + start >= arr.length) {
            return (T) arr[arr.length - 1 - start];
        }
        return (T) arr[idx];
    }

    public void add(T t) {
        if(size < arr.length) {
            arr[size++] = t;
        } else {
            if(start == arr.length) {
                start = 0;
            }
            arr[start++] = t;
        }
    }

    public int size() {
        return arr.length;
    }
}
