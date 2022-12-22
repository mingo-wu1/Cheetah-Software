# 舞蹈动作dat文件

## 1.如何生成舞蹈动作的dat文件

采用octave生成，将生成文件的名称、q矩阵和t矩阵作为参数输入，即可得到舞蹈dat文件。
q矩阵为6行n列矩阵，q矩阵的6行依次代表X、Y、Z、Roll、Pitch、Yaw（XYZ单位为m，RPY单位为弧度）。t矩阵为1行n列矩阵，代表时间，单位为s。q矩阵和t矩阵列数相同。
生成代码如下：

```octave
function []=getBalanceDanceDataDat(fileName,q_all,t)
  [m,n]=size(q_all);
  if m!=6
    print("q_all rows error!");
  else
    t_out = t(1,1:n);
    q_all_copy = q_all;
    fid=fopen(fileName,'w');
    fwrite(fid,"BZL01",	'char*1');
    fwrite(fid, m+1,	'uint32');
    fwrite(fid, n,	'uint32');
    fwrite(fid, q_all_copy,	'double');
    fwrite(fid, t_out,	'double');
    fclose(fid);
  end
end
```



## 2.如何添加舞蹈动作的dat文件

① 将生成的dat文件复制到工程中的config/dance/文件目录下
② 编辑工程中的config/dance/dance.txt在尾行加入舞蹈序号+英文空格+文件名称。
例如：文件名称为balance_stand_dance_2.dat，舞蹈序号为2（舞蹈序号自己制定，但不能与已存在的舞蹈序号一样）

dance.txt原始文本内容为：
0 balance_stand_dance_0.dat
1 balance_stand_dance_1.dat

在dance.txt在尾行加入的文本就是
2 balance_stand_dance_2.dat

修改后的dance.txt内容为：
0 balance_stand_dance_0.dat
1 balance_stand_dance_1.dat
2 balance_stand_dance_2.dat

## 3.舞蹈动作dat文件构成（选择性了解）

dat文件构成依次是char型“BZL01”字符串，uint32型矩阵行数（舞蹈动作文件行数为7 前六行为q矩阵，第七行是时间），uint32型矩阵列数，double类型的数据集合（该集合double类型数据个数等于行数乘以列数）

例如：q矩阵为[1 2 3 4 5 6]的转置，t为7，最终需要生成的double数据块为7行1列的数据。生成出二进制文件的dat文件共69位，以十六进制显示如下：

第01到16位：42 5A 4C 30 31 07 00 00 00 01 00 00 00 00 00 00
第17到32位：00 00 00 F0 3F 00 00 00 00 00 00 00 40 00 00 00
第33到48位：00 00 00 08 40 00 00 00 00 00 00 10 40 00 00 00
第49到64位： 00 00 00 14 40 00 00 00 00 00 00 18 40 00 00 00
第65到69位：00 00 00 1C 40

前五位：42 5A 4C 30 31对应的是“BZL01”;第6到9位：07 00 00 00，对应uint32的7;第10到13位： 01 00 00 00对应uint32类型的1；第14位到第69位每8位依次对应的是double 类型的1、2、3、4、5、6、7。

