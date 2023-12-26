#include <Python.h>//调用python脚本必修导入该头文件
#include <iostream>

int main() {
    //使用python之前，要调用Py_Initialize();这个函数进行初始化
    //Py_SetPath(L"D:\\software\\Miniconda3\\envs\\opencv\\Lib;D:\\software\\Miniconda3\\envs\\opencv\\Lib\\site-packages;D:\\software\\Miniconda3\\envs\\opencv\\DLLs;D:\\software\\Miniconda3\\envs\\opencv\\Library\\bin");
    Py_SetPythonHome(L"D:\\software\\Miniconda3\\envs\\opencv");
    Py_Initialize();
 
    //导入环境变量
    PyRun_SimpleString("import sys");
    //PyRun_SimpleString("sys.path.append('D:\\software\\Miniconda3\\envs\\opencv\\Lib')");
    //PyRun_SimpleString("sys.path.append('D:\\software\\Miniconda3\\envs\\opencv\\DLLs')");
    //PyRun_SimpleString("import zlib");
    //PyRun_SimpleString("from PIL import Image");
    //PyRun_SimpleString("import numpy as np");
    //python脚本路径
    PyRun_SimpleString("sys.path.append('./')"); //放在cpp的同一路径下
 
    PyObject* pModule = NULL;
    PyObject* pFunc = NULL;
 
    //PySys_SetPath(L"E:\\c_project\\C-tools\\CWithPython\\bin\\Release");
    pModule = PyImport_ImportModule("mytest");//此出的mytest是python脚本的名称
    
    pFunc = PyObject_GetAttrString(pModule, "func");//这里是要调用的函数名
    PyObject_CallObject(pFunc, NULL);

    //调用Py_Finalize，这个根Py_Initialize相对应的。
    Py_Finalize();
 
    return 0;
 
}