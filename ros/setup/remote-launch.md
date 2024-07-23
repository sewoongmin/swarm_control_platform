# Remote Launch

## Hostname setup

[여기 ](http://wiki.ros.org/ROS/NetworkSetup)참고

```
sudo gedit /etc/hosts
```

위 명령어를 통해 hosts 에 아래와 같이 멀티 머신들을 등록해 준다.&#x20;

{% tabs %}
{% tab title="hosts" %}
```
127.0.0.1	localhost
127.0.1.1	brontes
192.168.0.20	threewatt

# The following lines are desirable for IPv6 capable hosts
::1     ip6-localhost ip6-loopback
fe00::0 ip6-localnet
ff00::0 ip6-mcastprefix
ff02::1 ip6-allnodes
ff02::2 ip6-allrouters
```
{% endtab %}
{% endtabs %}



### 5.Timing issues, TF complaining about extrapolation into the future? <a href="#timing_issues.2c_tf_complaining_about_extrapolation_into_the_future.3f" id="timing_issues.2c_tf_complaining_about_extrapolation_into_the_future.3f"></a>

You may have a discrepancy in system times for various machines. You can check one machine against another using

```
ntpdate -q other_computer_ip
```

If there is a discrepancy, install chrony (for Ubuntu, **sudo apt-get install chrony**) and edit the chrony configuration file (**/etc/chrony/chrony.conf**) on one machine to add the other as a server. For instance, on the PR2, computer c2 gets its time from c1 and thus has the following line:

```
server c1 minpoll 0 maxpoll 5 maxdelay .05
```

That machine will then slowly move its time towards the server. If the discrepancy is enormous, you can make it match instantly using

```
/etc/init.d/chrony stop
ntpdate other_computer_ip
/etc/init.d/chrony start
```
