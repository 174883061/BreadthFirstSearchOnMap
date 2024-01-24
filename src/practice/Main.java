package practice;

import java.util.HashMap;

public class Main {
    public static void main(String[] args) {
        // 创建一个 HashMap
        HashMap<String, Integer> map = new HashMap<>();

        // 添加键值对
        map.put("key1", 1);
        map.put("key2", 2);

        // 打印当前 HashMap 的内容
        System.out.println("Original map: " + map);

        // 修改 key1 对应的值
        map.put("key1", 10);

        // 再次打印修改后的 HashMap 的内容
        System.out.println("Updated map: " + map);
    }
}