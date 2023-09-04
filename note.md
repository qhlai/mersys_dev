```bash
rosrun rqt_topic rqt_topic

rosrun rqt_image_view rqt_image_view
```


sudo add-apt-repository ppa:borglab/gtsam-release-4.0
sudo apt install libgtsam-dev libgtsam-unstable-dev

https://mp.weixin.qq.com/s?__biz=MzU1NjEwMTY0Mw==&mid=2247516753&idx=2&sn=d45869d8ee7e1d4b1c8ac04adbdcbb98&chksm=fbc8fb35ccbf72232aab1c664210aa005e9aec9c52f74fe8e03408f2cced0e0b21eebdda1594&scene=27



sensor_msgs::PointCloud2是ROS中常用的点云消息格式，其中包含point_step和row_step两个字段。

point_step表示每个点（point）的字节数，包括所有维度和通道（channel）的数据。例如，如果一个点包含三个浮点数（x、y和z）和一个颜色通道（rgb），而每个浮点数需要4个字节存储，颜色通道需要3个字节存储，则该点的point_step为4

* 3 + 3 = 15字节。

row_step表示每行点云数据的字节数，包括所有点和其他元素的数据。点云数据可以看作是二维数组，其中每行表示一个点云数据，而每个点的大小由point_step定义。因此，row_step等于每行点的数量乘以point_step。例如，如果点云数据包含1000个点，每个点的point_step为15字节，则row_step为1000

* 15 = 15000字节。

在ROS中，point_step和row_step用于指定点云消息的数据结构，以便发送和接收方能够正确地解析和处理点云数据。在使用sensor_msgs::PointCloud2消息时，必须正确设置point_step和row_step字段，以确保消息的正确性和完整性。

Cereal是一个用于序列化和反序列化C++对象的库，它可以将C++对象转换为二进制流或JSON格式，并可用于在不同平台和应用程序之间传输数据。在Cereal中，可以使用压缩算法来压缩序列化后的数据，从而减少数据传输的大小。

以下是使用Cereal进行序列化和压缩的示例代码：

arduino

```
#include <iostream>
#include <sstream>
#include <cereal/archives/binary.hpp>
#include <cereal/archives/json.hpp>
#include <cereal/types/vector.hpp>
#include <zlib.h>

struct MyData {
  int id;
  std::string name;
  std::vector<double> values;

  // 序列化函数
  template<class Archive>
  void serialize(Archive & archive) {
    archive(id, name, values);
  }
};

int main() {
  // 创建MyData对象
  MyData data = {1, "my_data", {1.0, 2.0, 3.0, 4.0, 5.0}};

  // 序列化MyData对象为二进制流
  std::stringstream ss_binary;
  cereal::BinaryOutputArchive ar_binary(ss_binary);
  ar_binary(data);

  // 压缩序列化后的二进制流
  std::string compressed_binary;
  uLongf compressed_size = compressBound(ss_binary.str().size());
  compressed_binary.resize(compressed_size);
  compress(reinterpret_cast<Bytef*>(&compressed_binary[0]), &compressed_size, reinterpret_cast<const Bytef*>(ss_binary.str().data()), ss_binary.str().size());
  compressed_binary.resize(compressed_size);

  // 序列化MyData对象为JSON格式
  std::stringstream ss_json;
  cereal::JSONOutputArchive ar_json(ss_json);
  ar_json(data);

  // 压缩序列化后的JSON数据
  std::string compressed_json;
  uLongf compressed_json_size = compressBound(ss_json.str().size());
  compressed_json.resize(compressed_json_size);
  compress(reinterpret_cast<Bytef*>(&compressed_json[0]), &compressed_json_size, reinterpret_cast<const Bytef*>(ss_json.str().data()), ss_json.str().size());
  compressed_json.resize(compressed_json_size);

  // 输出压缩后的数据
  std::cout << "Compressed binary size: " << compressed_binary.size() << std::endl;
  std::cout << "Compressed JSON size: " << compressed_json.size() << std::endl;

  return 0;
}
```

在上述代码中，我们首先创建了一个包含整数、字符串和向量的MyData对象，并使用Cereal将其序列化为二进制流和JSON格式。然后，我们使用zlib库中的compress函数对序列化后的数据进行压缩。最后，我们输出压缩后的数据大小。

需要注意的是，压缩算法可以减少序列化后数据的大小，但也会增加处理时间和计算负载。在实际应用中，需要根据具体问题进行调整，并在数据传输和处理的效率之间进行权衡。

sudo iftop -i lo -f "port 9033"

```
sudo iftop -i lo -f "port 9033"
```

3MB/s

下面是一个使用Cereal进行序列化、压缩和解压缩的示例代码：

arduino

```cpp
#include <iostream>
#include <sstream>
#include <cereal/archives/binary.hpp>
#include <zlib.h>

struct MyData {
  int id;
  std::string name;
  std::vector<double> values;

  // 序列化函数
  template<class Archive>
  void serialize(Archive & archive) {
    archive(id, name, values);
  }
};

int main() {
  // 创建MyData对象
  MyData data = {1, "my_data", {1.0, 2.0, 3.0, 4.0, 5.0}};

  // 序列化MyData对象为二进制流
  std::stringstream ss_binary;
  cereal::BinaryOutputArchive ar_binary(ss_binary);
  ar_binary(data);

  // 压缩序列化后的二进制流
  std::string compressed_binary;
  uLongf compressed_size = compressBound(ss_binary.str().size());
  compressed_binary.resize(compressed_size);
  compress(reinterpret_cast<Bytef*>(&compressed_binary[0]), &compressed_size, reinterpret_cast<const Bytef*>(ss_binary.str().data()), ss_binary.str().size());
  compressed_binary.resize(compressed_size);

  // 解压缩序列化后的二进制流
  std::string decompressed_binary;
  uLongf decompressed_size = ss_binary.str().size();
  decompressed_binary.resize(decompressed_size);
  uncompress(reinterpret_cast<Bytef*>(&decompressed_binary[0]), &decompressed_size, reinterpret_cast<const Bytef*>(compressed_binary.data()), compressed_binary.size());
  decompressed_binary.resize(decompressed_size);

  // 将解压缩后的二进制流反序列化为MyData对象
  MyData data_decompressed;
  std::stringstream ss_decompressed(decompressed_binary);
  cereal::BinaryInputArchive ar_binary_decompressed(ss_decompressed);
  ar_binary_decompressed(data_decompressed);

  // 输出反序列化后的MyData对象
  std::cout << "ID: " << data_decompressed.id << std::endl;
  std::cout << "Name: " << data_decompressed.name << std::endl;
  std::cout << "Values: ";
  for (auto value : data_decompressed.values) {
    std::cout << value << " ";
  }
  std::cout << std::endl;

  return 0;
}
```

在上述代码中，我们首先创建了一个包含整数、字符串和向量的MyData对象，并使用Cereal将其序列化为二进制流。然后，我们使用zlib库中的compress函数对序列化后的数据进行压缩，并使用uncompress函数对压缩后的数据进行解压缩。接下来，我们将解压缩后的二进制流反序列化为MyData对象，并输出其值。

需要注意的是，压缩算法可以减少序列化后数据的大小，但也会增加处理时间和计算负载。在实际应用中，需要根据具体问题进行调整，并在数据传输和处理的效率之间进行权衡。

可以使用`std::stringstream`来存储zlib压缩后的二进制流数据。`std::stringstream`是一个用于处理字符串数据的C++标准库类，它可以将字符串数据当作输入或输出流来处理。

在使用zlib进行压缩时，可以先将数据序列化为二进制流，并将其放入`std::stringstream`中。然后，使用zlib对`std::stringstream`中的数据进行压缩，并将压缩后的数据存储在另一个`std::stringstream`中。

以下是使用`std::stringstream`存储zlib压缩后的二进制流数据的示例代码：

arduino

```cpp
#include <iostream>
#include <sstream>
#include <zlib.h>

int main() {
  // 创建一个std::stringstream对象，并将数据序列化为二进制流
  std::stringstream ss;
  int data = 123;
  ss.write(reinterpret_cast<const char*>(&data), sizeof(data));

  // 压缩std::stringstream中的数据
  std::stringstream ss_compressed;
  uLongf compressed_size = compressBound(ss.str().size());
  ss_compressed.str(std::string(compressed_size, '\0'));
  compress(reinterpret_cast<Bytef*>(&ss_compressed.str()[0]), &compressed_size, reinterpret_cast<const Bytef*>(ss.str().data()), ss.str().size());
  ss_compressed.str().resize(compressed_size);

  // 输出压缩后的数据
  std::cout << "Compressed data size: " << ss_compressed.str().size() << std::endl;

  // 解压缩std::stringstream中的数据
  std::stringstream ss_decompressed;
  uLongf decompressed_size = ss.str().size();
  ss_decompressed.str(std::string(decompressed_size, '\0'));
  uncompress(reinterpret_cast<Bytef*>(&ss_decompressed.str()[0]), &decompressed_size, reinterpret_cast<const Bytef*>(ss_compressed.str().data()), ss_compressed.str().size());
  ss_decompressed.str().resize(decompressed_size);

  // 从std::stringstream中读取解压缩后的数据
  int data_decompressed;
  ss_decompressed.read(reinterpret_cast<char*>(&data_decompressed), sizeof(data_decompressed));

  // 输出解压缩后的数据
  std::cout << "Decompressed data: " << data_decompressed << std::endl;

  return 0;
}
```

在上述代码中，我们首先创建了一个`std::stringstream`对象，并将数据序列化为二进制流。然后，我们使用zlib对`std::stringstream`中的数据进行压缩，并将压缩后的数据存储在另一个`std::stringstream`中。接下来，我们使用zlib对压缩后的数据进行解压缩，并将解压缩后的数据存储在另一个`std::stringstream`中。最后，我们从`std::stringstream`中读取解压缩后的数据，并输出其值。

需要注意的是，在使用`std::stringstream`存储二进制流数据时，需要使用`reinterpret_cast`将指针类型转换为char类型的指针。另外，为了保证输出的数据正确，需要在读取数据时使用正确的类型和大小。
