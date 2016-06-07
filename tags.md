---
layout: page
title: Tags
---

<p>
{% for tag in site.tags %}
<a class="btn" href="{{ '/tags/#' }}{{ tag | first }}" >
	{{ tag.first }} ({{ tag | last | size }}) 
</a>
{% endfor %}
</p>

<hr>

<div>
{% for tag in site.tags %}
<a class="anchor" id="{{ tag.first }}"></a> <h4>{{ tag.first }}</h4>

	{% for post in site.tags[tag.first] %}
		<ul>
		    <li>
		        <a href="{{ post.url }}">{{ post.title }}</a> Posted on {{ post.date | date_to_string }}
		    </li>
		</ul>
	{% endfor %}

{% endfor %}
</div>