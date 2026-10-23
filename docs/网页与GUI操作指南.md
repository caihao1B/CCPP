# 董泽豪个人网站：从解压、本机查看到 GitHub 发布

版本：网页重设计版 / CCPP 2.1.0。适用：Windows 10 / 11。先看本机效果，再发布。**本交付没有替你上传 GitHub。**

## 0. 先分清两个压缩包

**Zehao_Portfolio_Redesigned.zip 是网站。** 包含 `index.html`、`ccpp.html`、`assets` 等。看网页不需要安装规划算法的依赖，不需要 ROS、Node.js 或 npm。电脑里已有 Python 3 即可使用附带的本地预览程序。

**CCPP_Mowing_Planner_v2.1.0.zip 是 Python 规划项目。** 用于修改地图、车辆、转弯半径、算法并重新计算；入口是项目根目录 `app.py`。它不是网站发布目录。

网站里切换的是已计算并复验的十组结果；不是浏览器现场求解。改了 Python 参数不会自动改变公网网页，必须重新导出网页数据并上传。

## 1. 解压并确认文件位置

1. 在浏览器下载网站 ZIP，下载完成后点浏览器的下载图标，再点“在文件夹中显示”。也可以按 **Windows 键 + E** 打开文件资源管理器，在左边点“下载”。
2. 找到 `Zehao_Portfolio_Redesigned.zip`，右键 → **全部解压**。不要在压缩包窗口中直接运行网页或脚本。
3. 解压位置可设为 `D:\robot_portfolio`。没有 D 盘，就用自己的文档目录；后面的路径必须换成实际路径。
4. 打开解压文件夹。如果第一层只有一个文件夹，再打开一层，直到看到下列文件。**同时看见 `index.html` 和 `assets` 才是网站根目录。**

```text
D:\robot_portfolio\
    index.html
    ccpp.html
    projects.html
    START_HERE.html
    START_PREVIEW.cmd
    preview_server.py
    assets\
    reports\
    .nojekyll
```

5. 在资源管理器上方的地址栏点一下，可以看见和复制完整目录路径。后面 PowerShell 需要的就是这个路径，不是 ZIP 文件的路径。
6. 推荐打开文件扩展名：Windows 11 在“查看 → 显示 → 文件扩展名”勾选。防止把文件保存为 `index.html.txt`。

## 2. 最容易的本机预览：双击启动

1. 在网站根目录双击 **`START_HERE.html`**，可以用浏览器阅读这份指南。
2. 接着双击 **`START_PREVIEW.cmd`**。它只启动本机预览，不上传任何内容，也不安装软件。
3. 会出现命令窗口，随后尝试自动打开浏览器。窗口中的典型文字是：

```text
PORTFOLIO LOCAL PREVIEW
Open this address in your browser:

  http://127.0.0.1:8000/

Keep this window open. Press Ctrl+C to stop.
```

4. 如果浏览器没有自动打开，先选中窗口显示的网址，复制；打开 Edge/Chrome，按 **Ctrl+L** 选中地址栏，粘贴网址，按 **Enter**。
5. 如果 8000 端口已经占用，预览程序会换一个空闲端口。**使用窗口实际打印的地址，不要坚持输入8000。**
6. 预览期间保留命令窗口。停止时点击该窗口，按 **Ctrl+C**。出现“请按任意键继续”时按任意键结束。关掉服务器后，已经打开的静态页面可能还显示，但刷新或新资源请求将失败。

这是本地地址，只在自己的电脑上使用，不能作为作品集网址发给面试官。也不需要为这个预览程序放开公网防火墙或使用管理员权限。

若提示没有 Python，请看下一节。若 Windows 弹出文件安全提示，先确认文件来源和代码内容；不需要关闭杀毒软件或降低系统安全策略。

## 3. PowerShell 从哪里打开？命令在哪里输入？

### 3.1 打开普通 PowerShell

1. 按键盘左下方 **Windows 键**，或者点任务栏的开始按钮。
2. 直接输入 `Windows PowerShell`。
3. 在搜索结果中选择 **Windows PowerShell → 打开**。本任务不用“以管理员身份运行”，也不用 PowerShell ISE。
4. 出现一个蓝色或黑色窗口，最后一行通常类似：

```text
PS C:\Users\你的用户名>
```

`PS ...>` 是系统的提示符，**不是你需要输入的命令**。把鼠标点在窗口里，在这一行后输入或粘贴命令，然后按 Enter。每次只执行下面的一行，确认输出正常后再进行下一行。不要把 Markdown 的三条反引号一起复制进去。

### 3.2 先进入网站文件夹

假设实际网站根目录是 `D:\robot_portfolio`，粘贴下面一行，按 Enter：

```powershell
Set-Location -LiteralPath "D:\robot_portfolio"
```

双引号保留，可以处理空格和中文路径。实际文件夹不同，就修改引号中的部分。

成功后提示符应变成：

```text
PS D:\robot_portfolio>
```

然后输入下一行并按 Enter：

```powershell
Get-ChildItem
```

屏幕应列出 `index.html`、`ccpp.html`、`assets` 等。如果只看到一个文件夹，说明还在外面一层；重新用 `Set-Location` 进入真正的网站根目录。出现“找不到路径”时，返回资源管理器复制地址栏路径，不要反复执行同一条错误路径。

### 3.3 检查 Python

输入并按 Enter：

```powershell
py --version
```

出现 `Python 3.x.x` 即可。若提示找不到 `py`，试下一行：

```powershell
python --version
```

如果第二条成功，后面所有开头的 `py` 换成 `python`。如果两条都失败，但旧规划项目能够运行，也可以使用那个项目的解释器，例如：

```powershell
& "D:\python_projects\CCPP_Mowing_Planner_v2.1.0\.venv\Scripts\python.exe" --version
```

这里前面的 **`&`** 用于运行带引号的程序路径；引号内必须是你真正存在的解释器。如果电脑没有 Python 3，需要先按 Python 官方 Windows 安装说明安装，再关闭、重新打开 PowerShell。不要安装叫作 `http.server` 的第三方包：它是 Python 自带模块。

### 3.4 手动启动网页服务

确定当前目录中直接有 `index.html` 后，输入下面一行并按 Enter：

```powershell
py -m http.server 8000 --bind 127.0.0.1
```

成功时会看到类似：

```text
Serving HTTP on 127.0.0.1 port 8000 ...
```

此时 PowerShell **没有回到新的 `PS ...>` 提示符是正常现象**，表示服务器仍在运行。不要继续在该窗口里输入其他命令，也不要以为卡住了。

打开浏览器，按 **Ctrl+L**，输入下面这个地址并按 Enter：

```text
http://127.0.0.1:8000/
```

要结束服务器，回到 PowerShell，按 **Ctrl+C**。若出现端口占用错误，把命令中的8000改成8001，浏览器地址也改成8001。手动的 `http.server` 不会自动替你选择端口。

## 4. 网站里怎样查看这次重点改动？

在首页点击 **“查看全覆盖规划演示”**。页面的叙事顺序是：

**同一田块 → 方格/三角格 → Greedy/MST–Euler/GA → 任务访问顺序 → 可行折线 → 受车辆约束的曲线与时间。**

### 4.1 先看六组算法与网格对比

页面第一张表有六组真实结果。单元面积固定0.36平方米，机器人、地图、随机种子和曲线策略保持一致。点击算法名称，会把下方交互案例切换为相应结果。

请同时看目标数、计划访问次数、原始折线长度、约束曲线长度、模拟时间和任务区面积覆盖率。相同单元面积不等于相同目标集合；不能只凭三角格路线短就说它全面优于方格。

### 4.2 看具体访问顺序

找到“一条访问顺序，三个观察层次”。先选择“② 节点访问顺序”。图上序号表示**第几次计划访问**，下面的表格提供对应的节点ID、坐标和回访次数。

“从第几次访问开始”输入框一次标注连续18次访问，以免整幅图被数字覆盖。完整顺序一直保留在表格和CSV中，不是只保存18个点。点击表格某一行，图上标注会从那次访问开始。

### 4.3 同一条顺序切换到曲线

点击“③ 曲线与机器人运动”，保持“叠加原始折线”勾选。先观察折线怎样被圆角替换，再按“看平滑弯道 → 播放”。速度建议1×，方便看到机器人位置与朝向同时变化。

### 4.4 专门检查180°反向访问

选择 **方格 · MST–Euler**，点击 **“看180°回环掉头”**，然后“播放”。窗口会显示完整的同点改向回环，而不是只展示其中一个圆弧。动画到片段终点会停止，不自动重置到初始姿态；点击“回到开头”才会手动重置。

这不是用额外角度插值把车体转过去。位置和航向来自同一条解析曲线，运动沿切线向前。所有新展示案例禁止车体原地旋转，也没有倒车运动。某些曲率跳变处会停车转舵，车体位置和航向不跳变。

**不要求每条掉头路线都长成“8”字。** 实际可用形状可能是Ω、泪滴或三圆弧复合回环。起终位姿、最小半径和无碰撞空间才是判定条件。如果这些候选都不合格，项目会拒绝，而不会缩小半径或瞬时旋转伪装成功。

### 4.5 比较车辆半径与时间优化

在案例下拉框选择前进车辆0.60m和0.90m两组。它们固定任务顺序与车体，改变最小半径。不要把它们与紧凑差速车组当成同场景品牌排名。

再看“固定访问顺序之后，再比较轨迹方案”表。圆弧、Bézier和保守时间三行保留相同任务顺序；Bézier几何不变时，时间优化与保守方案的时长可以直接对照。所有“时间”是模型预测，不是硬件运行计时。

## 5. 在 GitHub 上建立个人网站：全程浏览器操作

### 5.1 登录并核对用户名

1. 打开 GitHub 官网并登录。
2. 点击右上角头像，查看自己的用户名。
3. 下面以此前使用的 `caihao1B` 为例。用户名有大写时，个人网站仓库名使用对应的小写形式：`caihao1b.github.io`。
4. 若同名仓库已存在，先进入仓库，在 **Code → Download ZIP** 下载备份，不要未经检查就删除原网站。

### 5.2 创建仓库

1. GitHub右上角点 **“+” → New repository**。
2. Owner选择自己的账号。
3. Repository name填 `caihao1b.github.io`。使用其他账号时，必须替换成自己的用户名。
4. Description可以写 `Robotics and motion-planning portfolio`。
5. 选择 **Public**。公开仓库可以使用GitHub Free的Pages功能。
6. 可以勾选初始化README，点击 **Create repository**。

这只是在GitHub建立一个文件仓库，还没有完成网页发布。

### 5.3 上传网站内容，不是ZIP本身

1. 在仓库的 **Code** 页面点击 **Add file → Upload files**。如果是空仓库，页面也可能直接显示“uploading an existing file”链接。
2. 回到资源管理器，打开真正的网站根目录，确认能直接看到 `index.html`。
3. 拖入 `index.html`、`ccpp.html`、`projects.html`、`assets`文件夹、`reports`文件夹、`.nojekyll`、README等**目录内部的内容**。
4. 不要把ZIP直接传上去，也不要只传HTML而漏掉assets。`.cmd`和Python预览脚本可以保留在仓库中，但GitHub Pages不会执行它们。
5. 等网页文件列表完整显示，向下找到提交框，填写例如 `Publish robotics portfolio redesign`。
6. 有“Commit directly to main”选项时，选择直接提交main，点击 **Commit changes**。若选择了新分支，需将对应Pull Request合并到main后才能通过main发布。
7. 上传后回到仓库首页，确认层级是下面这样：

```text
caihao1b.github.io 仓库
    index.html           ← 根目录
    ccpp.html
    assets/
    reports/
    .nojekyll
```

如果变成 `仓库/Zehao_Portfolio/index.html`，而下一步发布目录选根目录，就可能看不到首页。应重新调整文件层级，不要在设置中选择一个不存在的自定义子目录。

网站文件名大小写必须一致。`.nojekyll`是空文件，如果拖放漏掉，可点Add file → Create new file，文件名填`.nojekyll`，提交即可。

### 5.4 开启Pages

1. 进入网站仓库，点击顶部 **Settings**。这不是账号个人设置，而是仓库自己的Settings。
2. 左边菜单找到 **Pages**。
3. 在 **Build and deployment** 下，Source选择 **Deploy from a branch**。
4. Branch选择 **main**。
5. 文件夹选择 **/ (root)**。
6. 点击 **Save**。

本网站已经是静态HTML/CSS/JS，不需要选择额外网页框架，也不需要购买域名、安装npm或创建Python服务器。

### 5.5 检查部署结果

1. 点击仓库顶部 **Actions**。
2. 打开最新Pages部署记录，检查是否成功。失败时点开失败步骤看具体日志，不要反复重新上传无关文件。
3. 回到 **Settings → Pages**，点击 **Visit site**。以这里显示的地址为准。
4. 使用示例账号时，预期网址是：

```text
https://caihao1b.github.io/
```

这才是能发给别人看的公网地址。首次发布或更新后，以部署记录完成状态为准；浏览器旧内容可按Ctrl+F5强制刷新。不要把本地 `127.0.0.1` 地址写在简历上。

### 5.6 发布前检查个人信息

确认中文姓名、英文拼写、教育起止时间、邮箱和GitHub链接。网站没有身份证件、家庭地址、电话或私人论文附件。主页头像位置目前使用ZD文字标识，不是生成的人像；可以以后换成自己的照片，不要用朋友的照片。

另外两个项目目前是有边界说明的概述页面，详细职责、截图、视频和代码证据仍待补充。不要将它们的示意框图描述为本轮已经测试的运行结果。

## 6. 修改网站内容：不用先学前端工程

### 6.1 在VS Code里打开

启动VS Code，点左上角 **文件 → 打开文件夹**，选择包含`index.html`的网站根目录，再点“选择文件夹”。左边资源管理器会显示文件。

点击 `index.html`，按Ctrl+F搜索姓名、邮箱或你要改的一句话，编辑后按 **Ctrl+S**。保持本机服务器窗口打开，回到浏览器按Ctrl+R查看更新。CSS修改不明显时按Ctrl+F5。

当前文件分工：`index.html`是首页，`ccpp.html`是覆盖项目，`projects.html`是另外两个项目概述；`assets/site.css`控制颜色和布局；`assets/replay.js`处理轨迹回放；`assets/data/cases.js`是实际计算数据。

不要手工改结果数字来“美化指标”。需要新结果时，重新计算和导出。

### 6.2 上传更新

本地修改满意后，回到网站仓库 **Add file → Upload files**，上传修改后的同路径文件，提交到main。修改了CSS只需上传CSS；重新导出了轨迹，则上传对应assets数据和媒体。不要把`.venv`、临时输出、API密钥或几十MB的Python项目ZIP传进网站目录。

文件数量较多时可分批上传；GitHub网页每次最多100个文件、单文件不超过25 MiB。本网站已控制在这些限制以内。

## 7. Python GUI的新版本怎么用？

1. 将项目ZIP解压到一个新目录，不覆盖旧版。例如最终根目录为 `D:\python_projects\CCPP_Mowing_Planner_v2.1.0`。
2. 在项目根目录找到并双击 `setup_windows.bat`。它创建独立`.venv`并安装依赖。不要直接使用旧ZIP中的虚拟环境。
3. 安装成功后双击 `run_gui.bat`。
4. 启动后默认选中 **08 | Square / GA / constrained forward loops**。按“Generate path + trajectory”实际计算；也可以点“Open saved result...”打开`comparison_examples/square_ga`目录。
5. 在Motion / curves页查看 **Turn treatment = loops_only**。这表示新展示模式只允许移动曲线，不允许车体原地转向。
6. 要观察同点掉头，加载`comparison_examples/square_mst`，点击 **Heading loop**，再点Play / pause。倍速保持1×。
7. 09是三角格GA，10是方格MST，11和12是前进车0.60m/0.90m。01—07明确标为旧版基线，可能仍包含合法的原地转向，不要混作新loops_only结果。

三个转向策略分别是：`legacy_pivot`保留旧版允许的原地转向；`loop_then_pivot`先尝试曲线、允许时明确回退；`loops_only`没有合格曲线就失败，不秘密回退。

### 用PowerShell启动GUI（备用）

按第3节打开普通PowerShell，输入下列第一行并按Enter，进入项目根目录；再输入第二行打开GUI：

```powershell
Set-Location -LiteralPath "D:\python_projects\CCPP_Mowing_Planner_v2.1.0"
```

```powershell
.\.venv\Scripts\python.exe app.py
```

这里运行的是**项目根目录的app.py**。不要直接点击运行`ccpp_mowing/__main__.py`，否则缺少CLI子命令可能产生SystemExit:2；真正错误原因仍应看终端前面的信息。

在VS Code使用F5时，先“文件 → 打开文件夹”打开项目根目录，按Ctrl+Shift+P搜索`Python: Select Interpreter`并选择`.venv\Scripts\python.exe`。再点左边“运行和调试”，选`1 GUI - v2.1 forward loops`，按F5。网站目录与Python项目目录不要混淆。

## 8. 高级：重算实验并更新网页

这部分不是首次查看网页的必要步骤。先保存当前结果、保持项目与网站两个目录独立。下列命令在**项目根目录PowerShell**执行，每条都按Enter；`.venv`须先安装完成。

先重算十组实验到一个新目录：

```powershell
.\.venv\Scripts\python.exe tools\build_comparison_suite.py --out comparison_rerun
```

该脚本不覆盖已有案例文件夹；目录已存在时用新目录名，避免误覆盖。

随后把新数据和图片导出到网站：

```powershell
.\.venv\Scripts\python.exe tools\export_comparison_site.py --runs comparison_rerun --site "D:\robot_portfolio" --no-video
```

`--no-video`不重新编码MP4。如果你改了输入，旧视频就不能继续当作新结果，必须重新生成或从页面移除；不能混用新表格和旧视频。完整重导出需要FFmpeg可执行程序在PATH中，可在同一终端用`ffmpeg -version`检查，再去掉`--no-video`重新执行。本网站已经包含视频，首次预览不需要安装FFmpeg。

重算和导出后运行项目测试，再本地查看网站，最后提交到GitHub。不要将本机生成器的运行过程描述成浏览器实时求解。

## 9. 常见问题

| 现象 | 怎么处理 |
|---|---|
| 双击启动窗口一闪而过 | 从已打开的PowerShell按第3节手动操作，阅读完整错误；确认Python存在 |
| 网页显示文件列表 | 当前服务目录没有根级index.html；Ctrl+C后进入正确目录再启动 |
| 404找不到页面 | 检查上传层级、main/root发布目录及大小写，查看Actions日志 |
| 页面只有文字没有样式 | assets目录漏传或目录层级改变；完整解压并保持相对位置 |
| “找不到路径” | 把资源管理器地址栏中的实际路径复制到Set-Location引号内 |
| 端口已被使用 | 一键脚本会换端口；手动命令改用8001，浏览器也改 |
| 浏览器打不开127.0.0.1 | 确认服务窗口还开着、启动成功、网址端口正确；这是本机地址，不是公网 |
| 模拟机器人看起来转得很快 | 用“看180°回环掉头”与1×；全任务100×不适合判断局部连续性 |
| 曲线生成报无安全解 | 不是忽略警告：检查半径、障碍、车辆尺寸和原访问路线。loops_only不会用原地旋转骗过约束 |
| 命令一直不结束 | 预览服务正常情况下会持续运行；按Ctrl+C停止。规划过程则看GUI进度/终端错误 |

## 10. 官方参考与验证边界

操作界面核对日期：2026-10-22。网页的桌面与移动布局、JavaScript轨迹数值、HTTP资源和视频已在Linux环境检查；没有声称在你的Windows电脑或已发布GitHub Pages上实测。

- Microsoft：启动PowerShell：https://learn.microsoft.com/en-us/powershell/scripting/windows-powershell/starting-windows-powershell
- GitHub：建立Pages网站：https://docs.github.com/en/pages/getting-started-with-github-pages/creating-a-github-pages-site
- GitHub：选择发布分支：https://docs.github.com/en/pages/getting-started-with-github-pages/configuring-a-publishing-source-for-your-github-pages-site
- GitHub：网页上传文件：https://docs.github.com/en/repositories/working-with-files/managing-files/adding-a-file-to-a-repository
- Python：http.server：https://docs.python.org/3/library/http.server.html
- Python：Windows使用说明：https://docs.python.org/3/using/windows.html

本地服务器仅用于自己预览，监听127.0.0.1，不作为面向公网的生产服务器。公开部署使用GitHub Pages。
