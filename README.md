# Smart-Car-Line-Follower

[![Smart-Car-Line-Follower](https://img.youtube.com/vi/6Vzyjh8USYs/0.jpg)](https://www.youtube.com/watch?v=6Vzyjh8USYs)

[View Video](https://www.youtube.com/watch?v=6Vzyjh8USYs)


My dad built a smart car robot and added down facing IR sensors to make it into a line follower. It worked fine but he wanted it to go faster so he switched from 2 LiPo batteries to 3 to make it faster.  It went faster, then got unstable.  He gave it to me to try to make it stable.

I changed the line detection algorithm, added settling logic for the PID's D term and added live tuning over bluetooth.  After tweaking the PID parameters, it works quite a bit better.
